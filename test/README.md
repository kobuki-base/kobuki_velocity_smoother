# Executing Tests

```bash
# run all tests in the current directory
$ pytest-3
# run all tests with full stdout (-s / --capture=no)
$ pytest-3 -s
# run a single file
$ pytest-3 -s test_translational_smoothing.py
# run a single test
$ pytest-3 -s test_translational_smoothing.py::test_publishing
# run using setuptools
$ python3 setup.py test
```
