# WSL Joystick Bridge

### Windows Admin Temrinal:
```bash
usbipd list
```

and find the controller `BUS_ID`

```bash
usbipd attach --wsl --busid <BUS_ID>
```