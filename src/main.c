#include <stdio.h>
#include <stdlib.h>
#include <tensorflow/c/c_api.h>
#include <string.h>

#include "../include/maps.h"
#include "../include/racetrack.h"
#include "../include/safeguard.h"
#include "../include/nn.h"

#define INPUT_SIZE 14
#define OUTPUT_SIZE 9

const int velocity_limit_x = 5;

const int velocity_limit_y = 5;

struct Map
{
    int width;
    int height;
    char **grid;
    /* number of goal positions */
    int ngoals;
    /* number of start positions */
    int nstarts;
    /* goal positions */
    Position **goals;
    /* start positions */
    Position **starts;
};

struct Position
{
    int x;
    int y;
};

struct Velocity
{
    int x;
    int y;
};

struct Acceleration
{
    int x;
    int y;
};

struct State
{
    Position *position;
    Velocity *velocity;
};

struct Distance
{
    int l1;
    int x;
    int y;
};

struct NNModel
{
    TF_Graph *graph;
    TF_Session *session;
    TF_Status *status;
};

struct NNInput
{
    TF_Tensor **values;
};


Acceleration *compute_acceleration(const Map *map, const State *state, const NNModel *nn_model,
                                   int look_ahead_steps, int safety_distance);

int look_ahead_check(const Map *map, const State *state, const NNModel *nn_model,
                     int look_ahead_steps, int safety_distance);

int is_valid_acceleration(const Map *map, const State *state, const Acceleration *acceleration);

int is_valid_velocity(const Map *map, const Position *position, const Velocity *velocity);

int is_valid_position(const Map *map, const Position *position);

int is_goal(const Map *map, const Position *position);

int is_zero(const Velocity *velocity);

Position *get_start_position(const Map *map);

Velocity *get_start_velocity();


int main()
{
    
    /* TODO command line interface */
    char *nn_model_filename = "../policies/corner/";

    Map *map = get_map();
    int step_limit = 50;
    int look_ahead_steps = 3;
    int safety_distance = 1;
    State *initial_state = get_intial_state(map);

    int success = run_safeguard_controller(map, initial_state, nn_model_filename, step_limit,
                                           look_ahead_steps, safety_distance);

    return success;
}

int run_safeguard_controller(const Map *map, const State *initial_state,
                             const char *nn_model_directory, int step_limit, int look_ahead_steps, int safety_distance)
{

    if (step_limit < 1)
    {
        return 0;
    }

    if (is_goal_state(map, initial_state))
    {
        return 1;
    }

    NNModel *nn_model = load_nn_model(nn_model_directory);
    if (nn_model == NULL)
    {
        return 0;
    }
	
    /* step zero */
    Acceleration *acceleration = compute_acceleration(map, initial_state, nn_model,
                                                      look_ahead_steps, safety_distance);
    State *state = execute_acceleration(map, initial_state, acceleration);
    delete_acceleration(acceleration);
    if (state == NULL)
    {
        delete_nn_model(nn_model);
        return 0;
    }

    int step = 1;
    State *next_state;
    while (!is_goal_state(map, state) && step < step_limit)
    {
        acceleration = compute_acceleration(map, state, nn_model, look_ahead_steps, safety_distance);
        next_state = execute_acceleration(map, state, acceleration);
        delete_acceleration(acceleration);
        delete_state(state);
        state = next_state;
        if (state == NULL)
        {
            delete_nn_model(nn_model);
            return 0;
        }
        step++;
    }

    delete_nn_model(nn_model);
    return 1;
}

Acceleration *compute_acceleration(const Map *map, const State *state, const NNModel *nn_model,
                                   int look_ahead_steps, int safety_distance)
{


    NNInput *nn_input = get_nn_input(map, state, nn_model);
    Acceleration *acceleration = call_nn_model(nn_model, nn_input);

    if (!look_ahead_check(map, state, nn_model, look_ahead_steps, safety_distance))
    {
        Acceleration *negated_acceleration = get_negated_acceleration(acceleration);
        delete_acceleration(acceleration);
        acceleration = negated_acceleration;
    }
    return acceleration;
}

int look_ahead_check(const Map *map, const State *state, const NNModel *nn_model,
                     int look_ahead_steps, int safety_distance)
{

    if (look_ahead_steps < 1)
    {
        return 1;
    }

    NNInput *nn_input = get_nn_input(map, state, nn_model);
    Acceleration *simulated_acceleration = call_nn_model(nn_model, nn_input);
    ;
    State *simulated_state = simulate_acceleration(map, state, simulated_acceleration);
    delete_nn_input(nn_input);
    delete_acceleration(simulated_acceleration);
    if (simulated_state == NULL)
    {
        return 0;
    }
    int safe = look_ahead_check(map, simulated_state, nn_model, look_ahead_steps - 1, safety_distance);
    delete_state(simulated_state);
    return safe;
}

State *simulate_acceleration(const Map *map, const State *state, const Acceleration *acceleration)
{
    return get_next_state(map, state, acceleration);
}

State *execute_acceleration(const Map *map, const State *state, const Acceleration *acceleration)
{
    State *next_state = get_next_state(map, state, acceleration);
    return next_state;
}

State *get_intial_state(const Map *map)
{
    Position *start_position = get_start_position(map);
    Velocity *start_velocity = get_start_velocity();
    State *initial_state = malloc(sizeof(State));
    initial_state->position = start_position;
    initial_state->velocity = start_velocity;
    return initial_state;
}

State *get_next_state(const Map *map, const State *state, const Acceleration *acceleration)
{
    if (!is_valid_acceleration(map, state, acceleration))
    {
        return NULL;
    }
    Position *position = state->position;
    Velocity *velocity = state->velocity;
    Velocity *next_velocity = malloc(sizeof(Velocity));
    Position *next_position = malloc(sizeof(Position));
    next_velocity->x = velocity->x + acceleration->x;
    next_velocity->y = velocity->y + acceleration->y;
    next_position->x = position->x + next_velocity->x;
    next_position->y = position->y + next_velocity->y;
    State *next_state = malloc(sizeof(State));
    next_state->position = next_position;
    next_state->velocity = next_velocity;
    return next_state;
}

int is_goal_state(const Map *map, const State *state)
{
    Position *position = state->position;
    Velocity *velocity = state->velocity;
    return is_goal(map, position) && is_zero(velocity);
}

Distance *get_goal_distance(const Map *map, const Position *position)
{
    Distance *goal_distance = malloc(sizeof(Distance));
    goal_distance->x = map->width;
    goal_distance->y = map->height;
    goal_distance->l1 = map->width + map->height;

    Position **goals = map->goals;
    for (int i = 0; i < map->ngoals; i++)
    {
        Position *goal = goals[i];
        int x = abs(position->x - goal->x);
        int y = abs(position->y - goal->y);
        int l1 = x + y;
        if (l1 < goal_distance->l1)
        {
            goal_distance->x = x;
            goal_distance->y = y;
            goal_distance->l1 = l1;
        }
    }
    return goal_distance;
}

Acceleration *get_negated_acceleration(const Acceleration *acceleration)
{
    Acceleration *negated_acceleration = malloc(sizeof(Acceleration));
    negated_acceleration->x = -acceleration->x;
    negated_acceleration->y = -acceleration->y;
    return negated_acceleration;
}

Position *get_position(const State *state)
{
    return state->position;
}

Velocity *get_velocity(const State *state)
{
    return state->velocity;
}

Acceleration *create_acceleration(int x, int y)
{
    Acceleration *acceleration = malloc(sizeof(Acceleration));
    acceleration->x = x;
    acceleration->y = y;
    return acceleration;
}

int get_wall_distance(const Map *map, const Position *position, const Velocity *velocity)
{
    int distance = 0;
    Position traversed_position = {position->x, position->y};
    while (is_valid_velocity(map, &traversed_position, velocity))
    {
        traversed_position.x += velocity->x;
        traversed_position.y += velocity->y;
        distance++;
    }
    return distance;
}

int is_valid_acceleration(const Map *map, const State *state, const Acceleration *acceleration)
{
    Position *position = state->position;
    Velocity *velocity = state->velocity;
    Velocity accumulated_velocity = {velocity->x + acceleration->x, velocity->y + acceleration->y};
    return is_valid_velocity(map, position, &accumulated_velocity);
}

int is_valid_velocity(const Map *map, const Position *position, const Velocity *velocity)
{
    int x = position->x;
    int y = position->y;
    int vx = velocity->x;
    int vy = velocity->y;

    if (vx > velocity_limit_x || vy > velocity_limit_y)
    {
        return 0;
    }

    int sign_vx = vx >= 0 ? 1 : -1;
    int sign_vy = vy >= 0 ? 1 : -1;
    int vx_abs = abs(vx);
    int vy_abs = abs(vy);

    for (int step_vx = 0; step_vx <= vx_abs; step_vx++)
    {
        for (int step_vy = 0; step_vy <= vy_abs; step_vy++)
        {
            if (velocity_to_traversed_positions[vx_abs][vy_abs][step_vx][step_vy])
            {
                Position traversed_position = {x + sign_vx * step_vx, y + sign_vy * step_vy};
                if (!is_valid_position(map, &traversed_position))
                {
                    return 0;
                }
            }
        }
    }
    return 1;
}

int is_valid_position(const Map *map, const Position *position)
{
    int width = map->width;
    int height = map->height;
    char **grid = map->grid;
    int x = position->x;
    int y = position->y;
    return x >= 0 && x < width && y >= 0 && y < height && grid[x][y] != 'x';
}

int is_goal(const Map *map, const Position *position)
{
    int x = position->x;
    int y = position->y;
    char **grid = map->grid;
    return grid[x][y] == 'g';
}

int is_zero(const Velocity *velocity)
{
    int vx = velocity->x;
    int vy = velocity->y;
    return vx == 0 && vy == 0;
}

Position *get_start_position(const Map *map)
{
    int width = map->width;
    int height = map->height;
    char **grid = map->grid;
    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            if (grid[x][y] == 's')
            {
                Position *start_position = malloc(sizeof(Position));
                start_position->x = x;
                start_position->y = y;
                return start_position;
            }
        }
    }
    return NULL;
}

Velocity *get_start_velocity()
{
    Velocity *start_velocity = malloc(sizeof(Velocity));
    start_velocity->x = 0;
    start_velocity->y = 0;
    return start_velocity;
}


void delete_map(Map *map)
{
    free(map->grid);
    for (int i = 0; i < map->nstarts; i++)
    {
        free(map->starts[i]);
    }
    for (int i = 0; i < map->ngoals; i++)
    {
        free(map->goals[i]);
    }
    free(map->starts);
    free(map->goals);
    free(map);
}

void delete_position(Position *position)
{
    free(position);
}

void delete_velocity(Velocity *velocity)
{
    free(velocity);
}

void delete_acceleration(Acceleration *acceleration)
{
    free(acceleration);
}

void delete_distance(Distance *distance)
{
    free(distance);
}

void delete_state(State *state)
{
    delete_position(state->position);
    delete_velocity(state->velocity);
    free(state);
}

Map *get_map()
{
    int width = sizeof(MAP) / sizeof(MAP[0]);
    int height = sizeof(MAP[0]) / sizeof(MAP[0][0]);
    Map *map = malloc(sizeof(Map));
    map->width = width;
    map->height = height;
    map->nstarts = 0;
    map->ngoals = 0;
    int starts_size = 1;
    int goals_size = 1;
    map->starts = malloc(starts_size * sizeof(Position *));
    map->goals = malloc(goals_size * sizeof(Position *));
    char **grid = (char **)malloc(width * sizeof(char *));
    for (int x = 0; x < width; x++)
    {
        grid[x] = (char *)malloc(height * sizeof(char));
    }
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            grid[x][y] = MAP[x][y];
            if (grid[x][y] == 's')
            {
                if (map->nstarts == starts_size)
                {
                    starts_size *= 2;
                    map->starts = realloc(map->starts, starts_size * sizeof(Position *));
                }
                Position *start = malloc(sizeof(Position));
                start->x = x;
                start->y = y;
                map->starts[map->nstarts] = start;
                map->nstarts += 1;
            }
            if (grid[x][y] == 'g')
            {
                if (map->ngoals == goals_size)
                {
                    goals_size *= 2;
                    map->goals = realloc(map->goals, goals_size * sizeof(Position *));
                }
                Position *goal = malloc(sizeof(Position));
                goal->x = x;
                goal->y = y;
                map->goals[map->ngoals] = goal;
                map->ngoals += 1;
            }
        }
    }
    map->grid = grid;
    return map;
}

float *get_feature_values(const Map *map, const State *state)
{
    float *feature_values = malloc(14 * sizeof(int));
    Position *position = get_position(state);
    feature_values[0] = (float)position->x;
    feature_values[1] = (float)position->y;
    Velocity *velocity = get_velocity(state);
    feature_values[2] = (float)velocity->x;
    feature_values[3] = (float)velocity->y;
    int i = 4;
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            if (x == 0 && y == 0)
            {
                continue;
            }
            Velocity direction = {x, y};
            feature_values[i] = (float)get_wall_distance(map, position, &direction);
            i++;
        }
    }
    Distance *goal_distance = get_goal_distance(map, position);
    feature_values[12] = (float)goal_distance->x;
    feature_values[13] = (float)goal_distance->y;
    return feature_values;
}


NNModel *load_nn_model(const char *filename)
{

    /* computation graph */
    TF_Graph *graph = TF_NewGraph();
    /* holds error information */
    TF_Status *status = TF_NewStatus();
    /* options that can be passed at session creation */
    TF_SessionOptions *session_opts = TF_NewSessionOptions();
    TF_Buffer *run_opts = NULL;
    const char *tags = "serve";
    int tags_len = 1;

    TF_Session *session = TF_LoadSessionFromSavedModel(session_opts, run_opts, filename,
                                                       &tags, tags_len, graph, NULL, status);

    NNModel *nn_model = malloc(sizeof(NNModel));
    nn_model->graph = graph;
    nn_model->session = session;
    nn_model->status = status;
    return nn_model;
}

/* TODO free memory */
NNInput *get_nn_input(const Map *map, const State *state, const NNModel *nn_model)
{
    const int64_t dims[2] = {1, INPUT_SIZE};
    const size_t ndata = INPUT_SIZE * sizeof(float);
    float *feature_values = get_feature_values(map, state);

    

    TF_Tensor *tensor = TF_AllocateTensor(TF_FLOAT, dims, 2, ndata);
    memcpy(TF_TensorData(tensor), feature_values, ndata);
    TF_Tensor **input_values = malloc(sizeof(TF_Tensor *));
    input_values[0] = tensor;

    NNInput *nn_input = malloc(sizeof(NNInput));
    nn_input->values = input_values;
    return nn_input;
}

/* TODO free memory */
Acceleration *call_nn_model(const NNModel *nn_model, const NNInput *nn_input)
{

    TF_Operation *input_operation = TF_GraphOperationByName(nn_model->graph, "main/input");
    TF_Output input[1] = {input_operation, 0};

    TF_Operation *output_operation = TF_GraphOperationByName(nn_model->graph, "main/output/BiasAdd");
    TF_Output output[1] = {output_operation, 0};

    TF_Tensor **input_values = nn_input->values;

    TF_Tensor *output_values[1] = {NULL};

    TF_SessionRun(nn_model->session, NULL, input, input_values, 1, output, output_values, 1, NULL, 0, NULL, nn_model->status);

    float *q_values = (float *)malloc(OUTPUT_SIZE * sizeof(float));
    memcpy(q_values, TF_TensorData(output_values[0]), OUTPUT_SIZE * sizeof(float));

    int max_q_value_index = 0;
    float max_q_value = q_values[max_q_value_index];
    for (int i = 0; i < OUTPUT_SIZE; i++)
    {
        if (q_values[i] > max_q_value)
        {
            max_q_value = q_values[i];
            max_q_value_index = i;
        }
    }
    int ax = (max_q_value_index / 3) - 1;
    int ay = (max_q_value_index % 3) - 1;
    return create_acceleration(ax, ay);
}

void delete_nn_model(NNModel *nn_model)
{
    TF_DeleteGraph(nn_model->graph);
    TF_DeleteSession(nn_model->session, nn_model->status);
    TF_DeleteStatus(nn_model->status);
    free(nn_model);
}

/* TODO implement this function */
void delete_nn_input(NNInput *nn_input)
{
}
