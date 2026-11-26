# MCP example

A simulated UR5e served to an AI agent over MCP. The walkthrough is in
[docs/mcp.md](../../docs/mcp.md#first-run-in-simulation).

| File | What it is |
|---|---|
| `ur5e_sim.yaml` | The built-in `ur_ur5e` (toy driver, no MuJoCo) with the action tools on, up to 10 actions per call and a 0.05 rad step envelope |
| `mcp.json` | A project-scope MCP config for Claude Code that serves `ur5e_sim.yaml`; copy it to `.mcp.json` in the repository root |
| `client.py` | A Python MCP client with no LLM: lists the tools, describes the robot, bends the elbow by 0.04 rad, then holds |

Run these from the repository root, after `pip install -e '.[mcp]'` (Python 3.10+):

```bash
python examples/mcp/client.py             # try the tools without an LLM

cp examples/mcp/mcp.json .mcp.json        # Claude Code asks to approve it on the next start
claude
```
