from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
  
class RedirectHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        query_components = parse_qs(urlparse(self.path).query)
        auth_code = query_components.get('code')
        # box_token = query_components.get('state')
        
        if auth_code:
            self.wfile.write(b'COPY THIS CODE:\n\n' + self.path.encode()+b"\n")
            # self.wfile.write(b'Authorization code: '+ str(auth_code[0]).encode())
            # self.wfile.write(b'\nBox token: ' + str(box_token[0]).encode())

        self.wfile.write(b'\n\nAuthorization complete. You can close this window.')

def main():
    PORT = 5000  # Ensure this matches your redirect URI port
    server = HTTPServer(('localhost', PORT), RedirectHandler)
    print(f'Starting server at http://localhost:{PORT}')
    server.serve_forever()
    return
    
if __name__ == "__main__":
    main()
    

