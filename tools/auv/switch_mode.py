"""Deprecated: independent ROV/AUV firmware required."""
import sys
def main():
    print("会话模式切换已停用。请烧录独立固件并使用匹配服务。",file=sys.stderr)
    return 2
if __name__ == '__main__':
    raise SystemExit(main())
