"""Monitor page. The graph is served at the site root and at /graph."""

from __future__ import annotations

from flask import Flask, Response, jsonify, request


def create_monitor_app(page: str, snapshot, web_port: int = 8080) -> Flask:
    app = Flask(__name__)
    body = page.replace("__WEB_PORT__", str(int(web_port)))

    @app.get("/")
    @app.get("/graph")
    def graph_page():
        host = request.host.split(":")[0]
        return Response(body.replace("__WEB_HOST__", host), mimetype="text/html")

    @app.get("/api/graph")
    def api():
        return jsonify(snapshot())

    return app
