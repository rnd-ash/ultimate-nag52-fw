#!/bin/bash

# Do EGS51 headers
python convert.py egs51_ecus/can_data.txt egs51_ecus/src/ _EGS51
# Do EGS52 headers
python convert.py egs52_ecus/can_data.txt egs52_ecus/src/ _EGS52
# Do EGS53 headers
python convert.py egs53_ecus/can_data.txt egs53_ecus/src/ _EGS53