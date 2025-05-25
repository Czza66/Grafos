from fastapi import FastAPI, HTTPException
import osmnx as ox
import networkx as nx
from pyomo.environ import *
import matplotlib.pyplot as plt
import io
import base64

app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)

# Cargar grafo
print("Cargando grafo...")
G = ox.graph_from_place("Bogotá, Colombia", network_type="drive")

@app.post("/ruta-optima")
async def calcular_ruta(coordenadas: list[str]):
    try:
        if len(coordenadas) < 2:
            raise HTTPException(status_code=400, detail="Debes proporcionar al menos dos coordenadas.")

        # Parsear coordenadas
        puntos = []
        for s in coordenadas:
            partes = s.split(",")
            if len(partes) != 2:
                raise HTTPException(status_code=400, detail=f"Coordenada inválida: {s}")
            lat = float(partes[0].strip())
            lon = float(partes[1].strip())
            puntos.append((lat, lon))

        # Nodos más cercanos
        nodos = [ox.distance.nearest_nodes(G, lon, lat) for lat, lon in puntos]

        # Rutas entre puntos consecutivos
        ruta_total = []
        for i in range(len(nodos) - 1):
            tramo = nx.shortest_path(G, nodos[i], nodos[i + 1], weight='length')
            ruta_total += tramo[:-1]
        ruta_total.append(nodos[-1])

        # Arcos y distancias
        arcos = []
        distancias = {}
        for u, v in zip(ruta_total[:-1], ruta_total[1:]):
            if G.has_edge(u, v):
                d = G[u][v][0]["length"] / 1000  # km
                arcos.append((u, v))
                distancias[(u, v)] = d

        nodos_unicos = set(n for a in arcos for n in a)

        # Modelo Pyomo
        model = ConcreteModel()
        model.A = Set(initialize=arcos, dimen=2)
        model.N = Set(initialize=nodos_unicos)
        model.x = Var(model.A, domain=Binary)

        model.obj = Objective(expr=sum(distancias[i] * model.x[i] for i in model.A), sense=minimize)

        def flujo(model, n):
            if n == nodos[0]:
                return sum(model.x[i] for i in model.A if i[0] == n) == 1
            elif n == nodos[-1]:
                return sum(model.x[i] for i in model.A if i[1] == n) == 1
            else:
                return (sum(model.x[i] for i in model.A if i[0] == n) -
                        sum(model.x[i] for i in model.A if i[1] == n)) == 0

        model.flujo = Constraint(model.N, rule=flujo)

        solver = SolverFactory("cbc")
        result = solver.solve(model, tee=False)

        ruta_optima = [i for i in model.A if value(model.x[i]) == 1]

        if not ruta_optima:
            raise HTTPException(status_code=400, detail="No se encontró una ruta óptima.")

        nodos_ruta = [ruta_optima[0][0]] + [a[1] for a in ruta_optima]

        # Mapa base64
        fig, ax = ox.plot_graph_route(G, nodos_ruta, route_color="orange", node_size=0, show=False, close=False)
        buf = io.BytesIO()
        fig.savefig(buf, format="png")
        buf.seek(0)
        img_base64 = base64.b64encode(buf.read()).decode("utf-8")
        plt.close(fig)

        coordenadas_ruta = [f"{G.nodes[n]['y']},{G.nodes[n]['x']}" for n in nodos_ruta]
        google_maps_url = "https://www.google.com/maps/dir/" + "/".join(coordenadas_ruta)
        distancia_total_km = round(sum(distancias[i] for i in ruta_optima), 2)

        return {
            "imagen_base64": img_base64,
            "nodos_recorridos": coordenadas_ruta,
            "distancia_total_km": distancia_total_km,
            "google_maps_url": google_maps_url
        }

    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
