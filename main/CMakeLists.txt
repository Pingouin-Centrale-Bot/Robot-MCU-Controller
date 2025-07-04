idf_component_register(SRC_DIRS "src" "src/libs" "src/modules"
                    INCLUDE_DIRS "src")

set(COMPONENT_REQUIRES "TMCStepper")
set(COMPONENT_REQUIRES "FastAccelStepper")
set(COMPONENT_REQUIRES "TM1638plus")
set(COMPONENT_REQUIRES "LedcStepper")