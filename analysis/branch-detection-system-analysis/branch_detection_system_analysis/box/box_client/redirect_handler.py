from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs



class AuthHTTPServer(HTTPServer):
    def __init__(self, server_address, RequestHandlerClass):
        super().__init__(server_address, RequestHandlerClass)
        self.auth_code = None
        self.state_received = None
        return
    

class RedirectHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        query_components = parse_qs(urlparse(self.path).query)

        self.server.auth_code = query_components.get("code", [None])[0]
        self.server.state_received = query_components.get("state", [None])[0]

        if self.server.auth_code:
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(self.success_page(code=self.server.auth_code).encode())

        else:
            self.send_response(400)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(self.error_page().encode())

        return
    
    def success_page(self, code):
        return_string = f"""
            <!DOCTYPE html>
            <html lang="en">
            <head>
            <meta charset="UTF-8">
            <title>Box Authorization Complete</title>
            <style>
                body {{
                    font-family: Arial, sans-serif;
                    background: #f7f9fc;
                    display: flex;
                    justify-content: center;
                    align-items: center;
                    height: 100vh;
                    margin: 0;
                }}
                .container {{
                    background: white;
                    padding: 2rem 3rem;
                    border-radius: 12px;
                    box-shadow: 0 6px 20px rgba(0,0,0,0.1);
                    text-align: center;
                    max-width: 500px;
                }}
                h1 {{
                    color: #2E6BD6;
                    margin-bottom: 1rem;
                    font-size: 1.8rem;
                }}
                p {{
                    color: #444;
                    font-size: 1.1rem;
                    margin-bottom: 1.5rem;
                }}
                code {{
                    background: #eef2f9;
                    padding: 0.3rem 0.5rem;
                    border-radius: 6px;
                    display: inline-block;
                    font-size: 0.95rem;
                    color: #333;
                }}
                .small {{
                    margin-top: 2rem;
                    font-size: 0.85rem;
                    color: #777;
                }}
            </style>
            </head>
            <body>
            <div class="container">
                <h1>Authorization Successful</h1>
                <p>You may now close this window.</p>
                <p>Your device received the authorization code:</p>
                <code>{code}</code>
                <p class="small">Your application will complete authentication automatically.</p>
            </div>
            </body>
            </html>
            """
        return return_string
    
    def error_page(self):
        return_string = """
            <!DOCTYPE html>
            <html lang="en">
            <head>
            <meta charset="UTF-8">
            <title>Error</title>
            <style>
                body {
                    font-family: Arial, sans-serif;
                    background: #fff0f0;
                    display: flex;
                    justify-content: center;
                    align-items: center;
                    height: 100vh;
                    margin: 0;
                }
                .container {
                    background: white;
                    padding: 2rem 3rem;
                    border-radius: 12px;
                    box-shadow: 0 6px 20px rgba(0,0,0,0.1);
                    text-align: center;
                }
                h1 {
                    color: #d64545;
                }
            </style>
            </head>
            <body>
            <div class="container">
                <h1>Missing Authorization Code</h1>
                <p>Something went wrong. Try restarting the login process.</p>
            </div>
            </body>
            </html>
            """
        return return_string


def run_server(server: AuthHTTPServer, port: int) -> None:
    print(f"Starting server at http://localhost:{port}")
    server.serve_forever()
    return

def main():
    import threading
    PORT = 5000
    server = AuthHTTPServer(("localhost", PORT), RedirectHandler)

    server_thread = threading.Thread(
        target=run_server,
        args=(server, PORT),  # <-- pass positional args as a tuple
        daemon=True
    )
    server_thread.start()    
    return


if __name__ == "__main__":
    main()
