"""Audit repository Markdown links and generate its document catalogue."""
import argparse,os,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def documents():
 names=set(subprocess.check_output(['git','ls-files'],cwd=ROOT,text=True).splitlines())|set(subprocess.check_output(['git','ls-files','--others','--exclude-standard'],cwd=ROOT,text=True).splitlines())
 return sorted(ROOT/n for n in names if n.endswith('.md')and(ROOT/n).is_file())
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--write-index',action='store_true');a=p.parse_args();broken=[];paths=documents()
 for file in paths:
  s=file.read_text(encoding='utf-8-sig')
  for m in re.finditer(r'(?<!!)\[[^\]]+\]\(([^\s)]+)\)',s):
   target=m[1].split('#')[0]
   if not target or re.match(r'^[a-zA-Z]+:',target):continue
   if not(file.parent/target).exists():broken.append(f'{file.relative_to(ROOT)}:{s[:m.start()].count(chr(10))+1}: {target}')
 if a.write_index:
  lines=['# 全仓库文档索引','', '更新：2026-10-10。此索引覆盖 Git 管理及待提交的 Markdown 文件；部署和验收记录保留当时事实，不代表当前设备状态。','', '| 文档 | 类别 |','|---|---|']
  for file in paths:
   if file==ROOT/'docs/catalog.md':continue
   rel=file.relative_to(ROOT).as_posix();kind='历史归档' if '/archive/'in rel or rel.startswith('legacy/')else '验收/复盘证据' if '/reports/'in rel or '/reviews/'in rel or '/deployment/'in rel else '标定证据' if '/calibration/'in rel else '当前说明/模块入口'
   lines.append(f'| [{rel}]({os.path.relpath(file,ROOT/"docs").replace(chr(92),"/")}) | {kind} |')
  (ROOT/'docs/catalog.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
 print(f'Checked {len(paths)} Markdown files; {len(broken)} missing local link targets')
 for b in broken:print(b)
 return bool(broken)
if __name__=='__main__':raise SystemExit(main())
