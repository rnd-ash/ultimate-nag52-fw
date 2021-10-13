#!/bin/bash

# Do EGS52 headers
python convert.py egs52_ecus/can_data.txt egs52_ecus/src/ EGS52_MODE
# Do EGS53 headers
python convert.py egs53_ecus/can_data.txt egs53_ecus/src/ EGS53_MODE