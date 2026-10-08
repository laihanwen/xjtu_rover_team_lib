"""Reloadable local console view, using the existing controller on port 8767."""
import argparse
import ast
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8768)
    args=parser.parse_args()
    source=Path(__file__).with_name('trial_control_web.py')
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def respond(self,code,body,content_type):
            self.send_response(code)
            self.send_header('Content-Type',content_type)
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers();self.wfile.write(body)
        def do_GET(self):
            if self.path=='/':
                tree=ast.parse(source.read_text(encoding='utf-8'))
                html=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign)
                          and any(isinstance(t,ast.Name) and t.id=='HTML' for t in n.targets))
                return self.respond(200,html.encode('utf-8'),'text/html; charset=utf-8')
            self.proxy()
        def do_POST(self):
            if self.headers.get('Origin')!=f'http://127.0.0.1:{args.port}':
                return self.respond(403,b'{"error":"invalid origin"}','application/json')
            self.proxy()
        def proxy(self):
            allowed=('/state','/logs','/recording') if self.command=='GET' else ('/action','/recording')
            if self.path not in allowed:
                return self.respond(404,b'{}','application/json')
            try:
                body=None
                if self.command=='POST':
                    size=int(self.headers.get('Content-Length',0))
                    if not 0<size<=64:raise ValueError('body size')
                    body=self.rfile.read(size)
                request=Request('http://127.0.0.1:8767'+self.path,data=body,method=self.command,
                                headers={'Origin':'http://127.0.0.1:8767','Content-Type':'application/json'})
                try:
                    response=urlopen(request,timeout=2)
                except HTTPError as error:response=error
                with response:
                    self.respond(response.code,response.read(),response.headers.get('Content-Type','application/json'))
            except (ValueError,URLError,TimeoutError,OSError):
                self.respond(502,b'{"error":"controller unavailable"}','application/json')
    print(f'Console view http://127.0.0.1:{args.port}/; existing controller 8767',flush=True)
    ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()


if __name__=='__main__':main()
