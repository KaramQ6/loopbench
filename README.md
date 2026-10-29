# 🎯 LoopBench

> **The zero-hardware MCP server that benchmarks how well AI models tune control loops.**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Python: 3.10+](https://img.shields.io/badge/Python-3.10%2B-brightgreen.svg)](https://python.org)

LoopBench turns any Large Language Model into an autonomous control systems engineer — without requiring a single piece of breadboard or soldering.

---

## ⚡ Quickstart (Zero Hardware)

Run directly with `uvx` inside Claude Desktop, Cursor, or any MCP client:

```bash
uvx loopbench
```

Or install via pip:

```bash
pip install loopbench
```

---

## 🛠️ MCP Tools (Core v0.1)

1. `list_plants`: Inspect available physical plant models (DC Motor, Thermal Chamber, Drone Altitude).
2. `step_response`: Test control gains (`kp`, `ki`, `kd`) against a target plant and receive precise metrics & ASCII plots.
3. `benchmark_run`: Evaluate disturbance rejection under load.
4. `get_tuning_hints`: Retrieve baseline classical heuristics (Ziegler-Nichols, recommended gain envelopes).

---

## 🏆 The LLM Leaderboard

*Coming soon in Week 5: Automated benchmark pitting Claude 3.5 Sonnet, GPT-4o, and Gemini against challenging control problems.*
