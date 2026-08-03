# Fixed vs float regression test

Host-side check that the `float-version` hot-path math matches exact integer envelope formulas and reports drift vs `main`'s Q24 path.

## Build and run

```bash
g++ -std=c++17 -O2 -o compare compare.cpp && ./compare
```

Optional: override table size:

```bash
g++ -std=c++17 -O2 -DARRAY_SIZE=1024 -o compare compare.cpp && ./compare
```

## Pass criteria

- **PASS** when float index/output match exact integer math for all `delta < phase_ticks`.
- A **NOTE** is printed if index differs from `main`'s Q24 by up to one step at the 2 s boundary (expected).
