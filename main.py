from fastapi import FastAPI, HTTPException
import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
from pyomo.environ import *
import folium
import io
import base64

app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)

# Cargar el grafo de Bogotá
print("Cargando grafo...")
G = ox.graph_from_place("Bogotá, Colombia", network_type='drive')


@app.post("/ruta-optima")
async def ruta_optima(coordenadas: list[str]):
    try:
        # Convertir coordenadas a tuplas
        puntos = []
        for s in coordenadas:
            partes = s.split(",")
            if len(partes) != 2:
                raise ValueError(f"Formato inválido: {s}")
            lat = float(partes[0].strip())
            lon = float(partes[1].strip())
            puntos.append((lat, lon))

        # Obtener nodos más cercanos
        print("Obteniendo nodos...")
        nodos_cercanos = [ox.distance.nearest_nodes(G, lon, lat) for lat, lon in puntos]
        nodo_origen = nodos_cercanos[0]
        nodo_destino = nodos_cercanos[-1]

        # Calcular la ruta total con NetworkX
        ruta_total = []
        for i in range(len(nodos_cercanos) - 1):
            tramo = nx.shortest_path(G, nodos_cercanos[i], nodos_cercanos[i + 1], weight='length')
            ruta_total += tramo[:-1]
        ruta_total.append(nodos_cercanos[-1])

        # Arcos y distancias
        arcos = []
        distancias = {}
        for u, v in zip(ruta_total[:-1], ruta_total[1:]):
            d = G[u][v][0]['length'] / 1000
            arcos.append((u, v))
            distancias[(u, v)] = d
        nodos = set([n for a in arcos for n in a])

        # Modelo Pyomo
        model = ConcreteModel()
        model.A = Set(initialize=arcos, dimen=2)
        model.N = Set(initialize=nodos)
        model.x = Var(model.A, domain=Binary)

        model.obj = Objective(expr=sum(distancias[i] * model.x[i] for i in model.A), sense=minimize)

        def flujo_balance(model, n):
            if n == nodo_origen:
                return sum(model.x[i] for i in model.A if i[0] == n) == 1
            elif n == nodo_destino:
                return sum(model.x[i] for i in model.A if i[1] == n) == 1
            else:
                return (sum(model.x[i] for i in model.A if i[0] == n) -
                        sum(model.x[i] for i in model.A if i[1] == n)) == 0

        model.flujo = Constraint(model.N, rule=flujo_balance)

        # Resolver con CBC
        solver = SolverFactory('cbc')
        result = solver.solve(model, tee=True)

        ruta_optima = [i for i in model.A if value(model.x[i]) == 1]
        nodos_ruta = [ruta_optima[0][0]] + [a[1] for a in ruta_optima]

        distancia_total = sum(distancias[i] for i in ruta_optima)

        # Crear mapa con folium
        lat0, lon0 = G.nodes[nodos_ruta[0]]['y'], G.nodes[nodos_ruta[0]]['x']
        mapa = folium.Map(location=[lat0, lon0], zoom_start=12)

        for nodo in nodos_ruta:
            lat = G.nodes[nodo]['y']
            lon = G.nodes[nodo]['x']
            folium.CircleMarker(location=(lat, lon), radius=3, color='blue', fill=True).add_to(mapa)

        coords = [(G.nodes[n]['y'], G.nodes[n]['x']) for n in nodos_ruta]
        folium.PolyLine(coords, color="orange", weight=3, opacity=0.8).add_to(mapa)

        # Convertir mapa en base64
        img_data = mapa._to_png(5)
        img_base64 = base64.b64encode(img_data).decode("utf-8")

        return {
            "imagen_base64": img_base64,
            "nodos_recorridos": [f"{lat},{lon}" for lat, lon in coords],
            "distancia_total_km": round(distancia_total, 2)
        }

    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
