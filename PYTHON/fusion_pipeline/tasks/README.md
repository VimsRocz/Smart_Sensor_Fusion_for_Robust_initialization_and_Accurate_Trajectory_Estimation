# Independent task entry points

Each module fixes the requested task and lets the canonical pipeline resolve
its prerequisites. For example:

```bash
python -m fusion_pipeline.tasks.task_04 --dataset x001 --method TRIAD
python -m fusion_pipeline.tasks.task_05 --dataset x001 --method SVD
python -m fusion_pipeline.tasks.task_07 --dataset x001 --method Davenport
```

Use `--list-subtasks` to print the exact subtask and figure catalog for one
module. Task 4 includes 4.6, Task 5 includes 5.10, and Task 7 includes 7.6.
Input layouts, algorithm settings, and output options remain configurable with
the same JSON/YAML files accepted by `PYTHON/main.py`.
