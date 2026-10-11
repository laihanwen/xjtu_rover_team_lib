"""只读导入关键帧筛选，不改写工作簿。"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

NS={'s':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
FIELDS=['frame','time_sec','original_cone_count','original_yellow_side','original_tag_visible',
        'selected','junction_clear','cone_body_clear','base_clear','purpose','notes']

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workbook',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    report={'workbook_sha256':hashlib.sha256(args.workbook.read_bytes()).hexdigest(),'cameras':{}}
    with zipfile.ZipFile(args.workbook) as archive:
        strings=[]
        if 'xl/sharedStrings.xml' in archive.namelist():
            strings=[''.join(x.itertext()) for x in ET.fromstring(archive.read('xl/sharedStrings.xml')).findall('s:si',NS)]
        for camera,sheet in [('front',1),('down',2)]:
            rows=[]
            for row in ET.fromstring(archive.read(f'xl/worksheets/sheet{sheet}.xml')).findall('s:sheetData/s:row',NS):
                if int(row.attrib['r'])<5:continue
                cells={}
                for cell in row.findall('s:c',NS):
                    column=''.join(ch for ch in cell.attrib['r'] if ch.isalpha());value=cell.find('s:v',NS)
                    text=value.text if value is not None else ''
                    if cell.attrib.get('t')=='s' and text:text=strings[int(text)]
                    elif cell.attrib.get('t')=='inlineStr':text=''.join(cell.find('s:is',NS).itertext())
                    cells[column]=text or ''
                if cells.get('B')=='':continue
                record=dict(zip(FIELDS,[cells.get(chr(i),'') for i in range(66,77)]))
                if not record['frame']:continue
                record['frame']=int(float(record['frame']));record['time_sec']=float(record['time_sec'])
                rows.append(record)
            selected=[r for r in rows if r['selected']=='是']
            report['cameras'][camera]={'rows':len(rows),'selected':len(selected),
                'unreviewed_selection':sum(not r['selected'] for r in rows),
                'selected_clear_counts':{field:sum(r[field]=='是' for r in selected) for field in ['junction_clear','cone_body_clear','base_clear']},
                'records':rows}
            (args.output/f'{camera}-selected.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2),encoding='utf-8')
    (args.output/'selection-report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:{a:b for a,b in v.items() if a!='records'} for k,v in report['cameras'].items()},ensure_ascii=False))

if __name__=='__main__':main()
