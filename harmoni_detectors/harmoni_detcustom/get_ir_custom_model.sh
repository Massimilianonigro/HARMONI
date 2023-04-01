#!/bin/bash

echo "This script should be run from the HARMONI directory in order to place the models in a parallel directory"

mkdir -p harmoni_models/ir && cd harmoni_models/ir
gdown "https://drive.google.com/uc?id=1QeJRoUWbO1zvgUD7CqtIqjiie_i0Sn1f"
gdown "https://drive.google.com/uc?id=1gHiNSPWOajRHWd4dB_QHlmtb_UYdQIGv"

