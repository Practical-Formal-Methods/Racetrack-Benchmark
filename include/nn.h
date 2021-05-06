#ifndef NN_H
#define NN_H

// #include <tensorflow/c/c_api.h>

#include "racetrack.h"

typedef struct NNModel NNModel;

typedef struct NNInput NNInput;

/**
     * creates a neural network model from saved model format
     */
NNModel *load_nn_model(const char *filename);

/**
     * creates a neural network input from the specified state
     */
NNInput *get_nn_input(const Map *map, const State *state, const NNModel *nn_model);

/**
     * calls a neural network model with a neural network input and returns the 
     * output as an acceleration
     */
Acceleration *call_nn_model(const NNModel *nn_model, const NNInput *nn_input);

void delete_nn_model(NNModel *nn_model);

void delete_nn_input(NNInput *nn_input);

#endif