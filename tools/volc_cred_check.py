#!/usr/bin/env python3
"""豆包语音凭据自查（`code=3001 load grant` 专用）。

**症状**：调用语音合成时返回

    HTTP 401 {"code":3001,"message":"load grant: requested grant not found in SaaS storage"}

**含义**：服务端在 SaaS 存储里**找不到这个 (APP ID, Access Token) 组合**。
注意它**与音色完全无关**——换任何 voice_type 都是同一个错。

常见三种原因，这个脚本逐条排除：

1. **token 有空白/换行/只复制了一半** → 打印长度和首尾，并对比 strip 前后
2. **APP ID 和 Token 不属于同一个应用** → 用三种摆放方式（都带 / 只带头 / 只带体）分别试
3. **服务没在这个应用下开通** → 只能回控制台看，但脚本会给出提示

用法：

    echo '<access token>' > ~/.volc_token && chmod 600 ~/.volc_token

    .venv/bin/python tools/volc_cred_check.py --appid 1409423557
    .venv/bin/python tools/volc_cred_check.py --appid 1409423557 --token-file ~/.volc_token

**不会泄露密钥**：只打印长度、首 4 位、尾 4 位，中间一律打码。
每次尝试的文本是 "你好"（2 字），万一成功也只消耗 2 字额度。
"""

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ros2_ws" / "src" / "fwr_voice"))

from fwr_voice.tts import _post_json  # noqa: E402

ENDPOINT = "https://openspeech.bytedance.com/api/v1/tts"


def mask(value):
    if not value:
        return "(空)"
    if len(value) <= 8:
        return f"{value[:2]}***{value[-2:]}（长度 {len(value)}）"
    return f"{value[:4]}…{value[-4:]}（长度 {len(value)}）"


def describe(raw, label):
    stripped = raw.strip()
    print(f"  {label}: {mask(stripped)}")
    if raw != stripped:
        print(f"    ⚠️ 末尾/开头有空白（原始 {len(raw)} 字符 vs 去空白 {len(stripped)}）")
    if "\n" in raw:
        print("    ⚠️ 含换行符")
    if " " in stripped:
        print("    ⚠️ 中间含空格")
    return stripped


def attempt(name, appid, header_token, body_token, text="你好"):
    headers = {"Content-Type": "application/json"}
    if header_token is not None:
        headers["Authorization"] = f"Bearer;{header_token}"
    body = {
        "app": {"appid": appid, "token": body_token or "", "cluster": "volcano_tts"},
        "user": {"uid": "cred-check"},
        "audio": {"voice_type": "BV700_streaming", "encoding": "pcm"},
        "request": {"reqid": "cred-check", "text": text,
                    "text_type": "plain", "operation": "query"},
    }
    try:
        status, resp = _post_json(ENDPOINT, headers, body, timeout=20)
    except Exception as exc:
        print(f"  [{name}] 请求失败 {type(exc).__name__}: {exc}")
        return None
    try:
        obj = json.loads(resp)
        code = obj.get("code")
        message = str(obj.get("message", ""))[:110]
        data_len = len(obj.get("data") or "")
    except json.JSONDecodeError:
        code, message, data_len = "非JSON", resp[:110].decode("utf-8", "replace"), 0
    print(f"  [{name}] HTTP {status}  code={code}  audio={data_len}B  {message}")
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(description="豆包语音凭据自查")
    parser.add_argument("--appid", default=os.environ.get("FWR_VOLC_APPID", ""))
    parser.add_argument("--token-file", default="~/.volc_token")
    parser.add_argument("--token-env", default="FWR_VOLC_TOKEN")
    args = parser.parse_args(argv)

    if not args.appid:
        parser.error("--appid 必填（或设置 FWR_VOLC_APPID）")

    print("=" * 60)
    print("凭据形状")
    print("=" * 60)
    appid = describe(args.appid, "APP ID")

    raw_token = os.environ.get(args.token_env)
    if raw_token is not None:
        print(f"  来源: 环境变量 {args.token_env}")
        token = describe(raw_token, "Token")
    else:
        path = Path(args.token_file).expanduser()
        if not path.is_file():
            print(f"  ⚠️ 找不到 {path}")
            print("     请先把 Access Token 存进去：")
            print(f"     echo '<token>' > {args.token_file} && chmod 600 {args.token_file}")
            return 2
        raw_token = path.read_text(encoding="utf-8")
        print(f"  来源: 文件 {path}")
        token = describe(raw_token, "Token")

    if not token:
        print("\n❌ token 为空，无法继续")
        return 2
    if appid.isdigit() is False:
        print("\n⚠️ APP ID 通常是纯数字（如 1409423557），你给的不是")

    print()
    print("=" * 60)
    print("三种凭据摆放方式")
    print("=" * 60)
    results = {}
    results["都带"] = attempt("都带", appid, token, token)
    results["只带请求头"] = attempt("只带请求头", appid, token, None)
    results["只带请求体"] = attempt("只带请求体", appid, "", token)

    print()
    print("=" * 60)
    print("结论")
    print("=" * 60)
    if 3000 in results.values() or 0 in results.values():
        working = [k for k, v in results.items() if v in (0, 3000)]
        print(f"✅ 通了！可用的摆放方式：{'、'.join(working)}")
        print("   把这一结果告诉我，我按这个方式固定实现。")
        return 0

    print("❌ 三种摆放方式全部 `code=3001 load grant`，说明是**凭据本身**的问题，按顺序查：")
    print()
    print("  1. 【最常见】Token 没复制全")
    print("     控制台里点『显示』后，要选中**整串**再复制；")
    print("     粘贴后长度应该明显大于 16 位（一般几十位）。上面打印了长度，自己核一下。")
    print()
    print("  2. APP ID 和 Token 不是同一个应用")
    print("     豆包语音控制台可以有多个『应用』。确认你复制 Token 的页面")
    print("     和你看到 APP ID 的页面是**同一个应用**。")
    print("     换一个应用的话 APP ID 也会变——两个值必须成对。")
    print()
    print("  3. 服务没在这个应用下开通")
    print("     『开通管理』里显示『已开通』是账号级的；")
    print("     如果这个应用是新建的，可能还需要在『我的资源』里确认绑定关系。")
    print()
    print("  4. Token 已失效/被重置")
    print("     如果之前点过『重置』或这个 Token 很久没用，回控制台重新生成一个。")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
