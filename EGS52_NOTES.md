# Notes on behavior of stock EGS52 module in a W203

## CAN DATA

* GS_418 - Tx every 20ms
* GS_338 - Tx every 20ms
* GS_218 - Tx every 20ms

* 0x07E9 - Diag (KWP2000) Tx ID
* 0x07E1 - Diag (KWP2000) Rx ID

All 3 CAN Frames are transmitted at the same time



|    |In 1|In 2|In 3|In 4|In 5|
|:-: |:-: |:-: |:-: |:-: |:-: |
|To 1| -  |y3  |y3  |x   |x   |
|To 2|y3  | -  |y5  |x   |x   |
|To 3|x   |y5  | -  |y4  |y4  |
|To 4|x   |x   |y4  |-   |y5  |
|To 5|x   |x   |x   |y5  |-   |

