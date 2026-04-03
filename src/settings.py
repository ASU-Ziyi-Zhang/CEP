################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

### Enums type
from enum import Enum
from math import pi

class LightStatus(Enum):
    # Enums for light status - matches ANL Roadrunner
    GREEN=2
    AMBER=1
    RED=0

class LightType(Enum):
    # Enums for intersection type - Matches ANL Roadrunner definitions
    LIGHT=10
    STOP=20 

class ControlType(Enum):
    # Enums for controller type
    SAS = -3
    EXT = -2
    NONE = -1
    MOBIL = 0
    CAV = 1
    PCC = 2
    BASIC = 3
    HDV = 4

class MOBILStatus(Enum):
    # Enums for MOBIL status on its current driving pattern
    NONE=0
    FREE=1
    LOWUTIL=2
    TLINTAC=3

### Settings
# Simulation settings
# TEND = 300. # progressed time until simulation ends automatically [s]

# Vehicle parameter settings
VEHWIDTH = 1.90 # assumed external vehicle width [m]
VEHLENGTH = 4.20 # assumed external vehicle length [m]

# Road parameter settings
LANEWIDTH = 3.20 # width of lanes [m] - 3.2 or 3.7 is common in NA
LANEWIDTH_INV = 1./LANEWIDTH # Inverse of lane width - precalculated to avoid division in loops

# Sensor settings
RADAR_RANGE = 200. # [m] Reliable detection range from radar for ACC
VISION_RANGE = 200. # [m] Reliable vision range for human drivers

# Connectivity settings
CONN_RANGE = 450. # [m] Reliable connectivity range based on Chicago testing
V2V_CONN_ENABLED = True # True/False to enable or disable V2V intention-sharing communications
V2I_CONN_ENABLED = True # True/False to enable or disable V2I SPaT communications

### Constants
RAD2DEG = pi*180.