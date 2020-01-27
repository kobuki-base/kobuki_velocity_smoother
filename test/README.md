# Executing Tests

```bash
INSTALL_DIR=<path_to_install_space>
$ ros2 test ${INSTALL_DIR}/share/velocity_smoother/launch_tests/test_translational_smoothing.py
```

To enable external interaction with it (e.g. via ros2cli), then use the `--disable-isolation` argument.
