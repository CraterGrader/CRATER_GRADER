# Test
Unit tests are run using `colcon test`. Use the following steps:

1. Build the package and dependencies.
```
colcon build --packages-up-to planning
```

2. Run the tests.
```
colcon test --packages-select planning --event-handlers console_cohesion+
```