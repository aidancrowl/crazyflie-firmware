#define DEBUG_MODULE "ESTIMATOR"
#include "debug.h"

#include "cfassert.h"
#include "estimator.h"
#include "estimator_complementary.h"
#include "estimator_kalman.h"

#define DEFAULT_ESTIMATOR complementaryEstimator // Macro definition
static StateEstimatorType currentEstimator = anyEstimator; // StateEstimatorType is enum that lists estimator types (any, complementary, kalman)

static void initEstimator(const StateEstimatorType estimator);
static void deinitEstimator(const StateEstimatorType estimator);

typedef struct { // Struct that defines functions tied to specific estimator type
  void (*init)(void);
  void (*deinit)(void);
  bool (*test)(void);
  void (*update)(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
  const char* name;
  bool (*estimatorEnqueueTDOA)(const tdoaMeasurement_t *uwb);
  bool (*estimatorEnqueuePosition)(const positionMeasurement_t *pos);
  bool (*estimatorEnqueuePose)(const poseMeasurement_t *pose);
  bool (*estimatorEnqueueDistance)(const distanceMeasurement_t *dist);
  bool (*estimatorEnqueueTOF)(const tofMeasurement_t *tof);
  bool (*estimatorEnqueueAbsoluteHeight)(const heightMeasurement_t *height);
  bool (*estimatorEnqueueFlow)(const flowMeasurement_t *flow);
  bool (*estimatorEnqueueYawError)(const yawErrorMeasurement_t *error);
  bool (*estimatorEnqueueSweepAngles)(const sweepAngleMeasurement_t *angles);
} EstimatorFcns;

#define NOT_IMPLEMENTED ((void*)0)

static EstimatorFcns estimatorFunctions[] = {
    {
        .init = NOT_IMPLEMENTED,
        .deinit = NOT_IMPLEMENTED,
        .test = NOT_IMPLEMENTED,
        .update = NOT_IMPLEMENTED,
        .name = "None",
        .estimatorEnqueueTDOA = NOT_IMPLEMENTED,
        .estimatorEnqueuePosition = NOT_IMPLEMENTED,
        .estimatorEnqueuePose = NOT_IMPLEMENTED,
        .estimatorEnqueueDistance = NOT_IMPLEMENTED,
        .estimatorEnqueueTOF = NOT_IMPLEMENTED,
        .estimatorEnqueueAbsoluteHeight = NOT_IMPLEMENTED,
        .estimatorEnqueueFlow = NOT_IMPLEMENTED,
        .estimatorEnqueueYawError = NOT_IMPLEMENTED,
        .estimatorEnqueueSweepAngles = NOT_IMPLEMENTED,
    }, // Any estimator
    {
        .init = estimatorComplementaryInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorComplementaryTest,
        .update = estimatorComplementary,
        .name = "Complementary",
        .estimatorEnqueueTDOA = NOT_IMPLEMENTED,
        .estimatorEnqueuePosition = NOT_IMPLEMENTED,
        .estimatorEnqueuePose = NOT_IMPLEMENTED,
        .estimatorEnqueueDistance = NOT_IMPLEMENTED,
        .estimatorEnqueueTOF = estimatorComplementaryEnqueueTOF,
        .estimatorEnqueueAbsoluteHeight = NOT_IMPLEMENTED,
        .estimatorEnqueueFlow = NOT_IMPLEMENTED,
        .estimatorEnqueueYawError = NOT_IMPLEMENTED,
        .estimatorEnqueueSweepAngles = NOT_IMPLEMENTED,
    }, // Complementary estimator (I believe this is the one included with the original Crazyflie firmware?)
    {
        .init = estimatorKalmanInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorKalmanTest,
        .update = estimatorKalman,
        .name = "Kalman",
        .estimatorEnqueueTDOA = estimatorKalmanEnqueueTDOA,
        .estimatorEnqueuePosition = estimatorKalmanEnqueuePosition,
        .estimatorEnqueuePose = estimatorKalmanEnqueuePose,
        .estimatorEnqueueDistance = estimatorKalmanEnqueueDistance,
        .estimatorEnqueueTOF = estimatorKalmanEnqueueTOF,
        .estimatorEnqueueAbsoluteHeight = estimatorKalmanEnqueueAbsoluteHeight,
        .estimatorEnqueueFlow = estimatorKalmanEnqueueFlow,
        .estimatorEnqueueYawError = estimatorKalmanEnqueueYawError,
        .estimatorEnqueueSweepAngles = estimatorKalmanEnqueueSweepAngles,
    }, // Kalman filter/estimator, I believe added in by a third party developer (see estimator_kalman.c)
};

// Function: stateEstimatorInit
// Definition: Initialize selected estimator by calling stateEstimatorSwitchTo
// Parameters: StateEstimatorType estimator - enum that lists estimator types (any - 0, complementary - 1, kalman - 2) (estimator.h)
// Return Type: void

void stateEstimatorInit(StateEstimatorType estimator) {
  stateEstimatorSwitchTo(estimator);
}

// Function: stateEstimatorSwitchTo
// Definition: Switches to input estimator and initializes it by calling initEstimator()
// Parameters: StateEstimatorType estimator - enum that lists estimator types (any - 0, complementary - 1, kalman - 2) (estimator.h)
// Return Type: void

void stateEstimatorSwitchTo(StateEstimatorType estimator) {
  if (estimator < 0 || estimator >= StateEstimatorTypeCount) { // Return if input estimator parameter isn't within enum range
    return;
  }

  StateEstimatorType newEstimator = estimator; // Declare new estimator type basesd on input estimator parameter

  if (anyEstimator == newEstimator) { // If input estimator = 0, set to complementaryEstimator (DEFAULT_ESTIMATOR)
    newEstimator = DEFAULT_ESTIMATOR; // DEFAULT_ESTIMATOR = complementaryEstimator (1)
  }

  StateEstimatorType forcedEstimator = ESTIMATOR_NAME; // Where is ESTIMATOR_NAME defined??
  if (forcedEstimator != anyEstimator) {
    DEBUG_PRINT("Estimator type forced\n");
    newEstimator = forcedEstimator;
  }

  initEstimator(newEstimator); // Initialize newEstimator
  StateEstimatorType previousEstimator = currentEstimator; // Store previousEstimator
  currentEstimator = newEstimator; // Set currentEstimator to newEstimator
  deinitEstimator(previousEstimator); // Deinitialize previousEstimator

  DEBUG_PRINT("Using %s (%d) estimator\n", stateEstimatorGetName(), currentEstimator); // Print current estimator
}

// Function: getStateEstimator
// Definition: Returns number/name of currentEstimator enum
// Parameters: void
// Return Type: StateEstimatorType - enum that lists estimator types (any - 0, complementary - 1, kalman - 2) (estimator.h)

StateEstimatorType getStateEstimator(void) {
  return currentEstimator;
}

// Function: initEstimator
// Definition: Initializes input estimator by calling associated initialization function defined in EstimatorFcns estimatorFunctions[] array
// Parameters: const StateEstimatorType estimator - enum that lists estimator types (any - 0, complementary - 1, kalman - 2) (estimator.h)
// Return Type: static void

static void initEstimator(const StateEstimatorType estimator) {
  if (estimatorFunctions[estimator].init) { // If defined, i.e. not anyEstimator type
    estimatorFunctions[estimator].init(); // Call associated initialization function
  }
}

// Function: deinitEstimator
// Definition: Deinitializes input estimator by calling associated deinitialization function defined in EstimatorFcns estimatorFunctions[] array (not implemented)
// Parameters: const StateEstimatorType estimator - enum that lists estimator types (any - 0, complementary - 1, kalman - 2) (estimator.h)
// Return Type: static void

static void deinitEstimator(const StateEstimatorType estimator) {
  if (estimatorFunctions[estimator].deinit) { // If defined but it's not implemented so nothing happens here
    estimatorFunctions[estimator].deinit();
  }
}

// Function: stateEstimatorTest
// Definition: Tests input estimator by calling associated test function defined in EstimatorFcns estimatorFunctions[] array
// Parameters: void
// Return Type: bool

bool stateEstimatorTest(void) {
  return estimatorFunctions[currentEstimator].test(); // Call associated test function
}

// Function: stateEstimator
// Definition: Primary associated estimator function (this is where the ACTION happens) defined for each estimator type 
// Parameters:
//		state_t *state - struct reflecting current state of UAV (stabilizer_types.h)
//		sensorDatat_t *sensors - struct reflecting current sensor measurements (stabilizer_types.h)
//		control_t *control - struct reflecting current positional control inputs (roll, pitch, yaw, thrust) (stabilizer_types.h)
//		const uint32_t tick - unsigned int 
// Return Type: void

void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
  estimatorFunctions[currentEstimator].update(state, sensors, control, tick); // Call primary estimator function
}

const char* stateEstimatorGetName() {
  return estimatorFunctions[currentEstimator].name;
}


bool estimatorEnqueueTDOA(const tdoaMeasurement_t *uwb) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueTDOA) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueTDOA(uwb);
  }

  return false;
}

bool estimatorEnqueueYawError(const yawErrorMeasurement_t* error) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueYawError) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueYawError(error);
  }

  return false;
}

bool estimatorEnqueuePosition(const positionMeasurement_t *pos) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueuePosition) {
    return estimatorFunctions[currentEstimator].estimatorEnqueuePosition(pos);
  }

  return false;
}

bool estimatorEnqueuePose(const poseMeasurement_t *pose) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueuePose) {
    return estimatorFunctions[currentEstimator].estimatorEnqueuePose(pose);
  }

  return false;
}

bool estimatorEnqueueDistance(const distanceMeasurement_t *dist) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueDistance) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueDistance(dist);
  }

  return false;
}

bool estimatorEnqueueTOF(const tofMeasurement_t *tof) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueTOF) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueTOF(tof);
  }

  return false;
}

bool estimatorEnqueueAbsoluteHeight(const heightMeasurement_t *height) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueAbsoluteHeight) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueAbsoluteHeight(height);
  }

  return false;
}

bool estimatorEnqueueFlow(const flowMeasurement_t *flow) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueFlow) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueFlow(flow);
  }

  return false;
}

bool estimatorEnqueueSweepAngles(const sweepAngleMeasurement_t *angles) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueSweepAngles) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueSweepAngles(angles);
  }

  return false;
}
