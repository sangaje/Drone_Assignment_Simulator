import time

from tqdm.notebook import tqdm

EPOCHS = 5
STEPS  = 2000

p_epoch = tqdm(total=EPOCHS, desc="Epoch", position=0, leave=True, colour="cyan")
p_step  = tqdm(total=STEPS,  desc="Step ", position=1, leave=True, colour="green",
               bar_format="{l_bar}{bar}{r_bar}")

for e in range(EPOCHS):
    p_step.reset()
    start = time.perf_counter()
    for s in range(STEPS):
        # ... 작업 ...
        if s % 50 == 0:
            dt = max(time.perf_counter() - start, 1e-9)
            speed = (s+1) / dt
            p_step.set_postfix(loss=f"{0.0123:.4f}", speed=f"{speed:6.1f}/s")
        p_step.update(1); time.sleep(0.001)
    p_epoch.update(1)

p_step.close(); p_epoch.close()
