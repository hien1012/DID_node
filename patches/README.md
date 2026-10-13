# Local camera HAL change

`components/esp32-camera/` is ignored by Git in this workspace. The working copy
already contains the pause/resume change used by `main/cam_record.c`.

After restoring the same locally customized esp32-camera 2.1.7 component in a
fresh checkout, apply `esp32-camera-pause.patch` from the repository root with:

```powershell
git apply --ignore-whitespace patches/esp32-camera-pause.patch
```

The patch changes `cam_stop()` to wait for `cam_task` to stop DMA and recycle
pending frames before the upload starts. It is based on this project's local
camera driver and may need rebasing for a different driver version.
