# Test
Unit tests are run using `colcon test`. Run the following commands (from within the Docker container).

## Usage Instructions
1. Build the package and dependencies.
```
colcon build --packages-up-to planning
```

2. Run the tests.
```
colcon test --packages-select planning --event-handlers console_cohesion+
```

## Resources
- Colcon test "how-to": https://colcon.readthedocs.io/en/released/user/how-to.html
