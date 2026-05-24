### Bloonin

current build w camera demo (this will change)

```
cmake -S . -B build -DRUN_OV5640_DEMO=ON
cmake --build build
```

read serial with something like:  
```screen /dev/tty.usbmodem11301 115200```
