from task_optimizer_standalone import TaskOptimizer
opt = TaskOptimizer()
print("Starting optimization debug...")
path = opt.solve(0, 180)
print(f"Result path: {path}")
