import base64
import os
import subprocess
import sys
import time
from pathlib import Path

from anthropic import Anthropic


ROBOT_TOOLS = [
    {
        "name": "get_robot_mode",
        "description": "获取机器人当前的模式和状态。",
        "input_schema": {
            "type": "object",
            "properties": {}
        }
    },
    {
        "name": "set_robot_mode",
        "description": "设置机器人工作模式。可用缩写: LD(运动模式), SD(站立平衡), JD(关节锁定), DD(阻尼模式), PD(零力矩)。",
        "input_schema": {
            "type": "object",
            "properties": {
                "mode": {
                    "type": "string",
                    "description": "模式缩写，如 LD, SD, JD, DD, PD"
                }
            },
            "required": ["mode"]
        }
    },
]


class RobotAgent:
    def __init__(self):
        api_key = os.getenv("ANTHROPIC_AUTH_TOKEN")
        base_url = os.getenv("ANTHROPIC_BASE_URL")
        if not api_key:
            raise RuntimeError("请设置环境变量 ANTHROPIC_AUTH_TOKEN")

        self.client = Anthropic(api_key=api_key, base_url=base_url)
        self.model = os.getenv("AGENT_MODEL", "kimi-latest")
        self.history = []
        self._script_dir = os.path.dirname(os.path.abspath(__file__))

    def _run_script(self, *args) -> str:
        cmd = [sys.executable] + list(args)
        try:
            result = subprocess.run(
                cmd,
                cwd=self._script_dir,
                capture_output=True,
                text=True,
                timeout=30
            )
            output = result.stdout.strip()
            if result.returncode != 0 and result.stderr:
                output += f"\n[stderr] {result.stderr.strip()}"
            return output
        except subprocess.TimeoutExpired:
            return "执行超时"
        except Exception as e:
            return f"执行异常: {e}"

    def _call_tool(self, name: str, inputs: dict) -> str:
        if name == "get_robot_mode":
            return self._run_script("get_mode.py")

        if name == "set_robot_mode":
            mode = inputs.get("mode", "")
            return self._run_script("set_mode.py", mode)

        return f"未知工具: {name}"

    def chat(self, user_text: str) -> str:
        print(f"\n[User] {user_text}")
        self.history.append({"role": "user", "content": user_text})

        response = self.client.messages.create(
            model=self.model,
            max_tokens=1024,
            tools=ROBOT_TOOLS,
            messages=self.history
        )

        tool_use_blocks = [b for b in response.content if b.type == "tool_use"]
        text_blocks = [b for b in response.content if b.type == "text"]

        if not tool_use_blocks:
            reply = text_blocks[0].text if text_blocks else "..."
            print(f"[Agent] {reply}")
            self.history.append({"role": "assistant", "content": reply})
            return reply

        # Agent decided to use tool(s); handle the first one for simplicity
        block = tool_use_blocks[0]
        print(f"[Agent] 调用工具: {block.name}")
        result = self._call_tool(block.name, block.input)
        print(f"[Result] {result}")

        # Append tool_use and tool_result per Anthropic protocol
        self.history.append({"role": "assistant", "content": response.content})
        self.history.append({
            "role": "user",
            "content": [{
                "type": "tool_result",
                "tool_use_id": block.id,
                "content": result,
            }]
        })

        second = self.client.messages.create(
            model=self.model,
            max_tokens=1024,
            tools=ROBOT_TOOLS,
            messages=self.history
        )
        reply = second.content[0].text if second.content else "..."
        print(f"[Agent] {reply}")
        self.history.append({"role": "assistant", "content": reply})
        return reply


def main():
    try:
        agent = RobotAgent()
    except RuntimeError as e:
        print(e)
        sys.exit(1)

    print("=" * 40)
    print("Robot Agent 已启动，输入 'exit' 退出")
    print("=" * 40)

    # Demo: 你可以直接输入类似的话来复现以下演示
    demos = [
        "看看机器人现在是什么模式",
        "设置机器人模式为站立平衡 SD模式"
    ]

    for text in demos:
        agent.chat(text)
        time.sleep(3)

    while True:
        sys.stdout.write("\n> ")
        sys.stdout.flush()
        try:
            text = sys.stdin.readline().strip()
        except UnicodeDecodeError:
            text = sys.stdin.buffer.readline().decode("utf-8", errors="replace").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not text or text.lower() in ("exit", "quit"):
            break
        agent.chat(text)

    print("再见！")


if __name__ == "__main__":
    main()
