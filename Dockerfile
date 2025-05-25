FROM python:3.11-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y coinor-cbc build-essential libgeos-dev && \
    pip install --no-cache-dir --upgrade pip && \
    pip install fastapi uvicorn osmnx networkx matplotlib folium pyomo scipy

WORKDIR /app
COPY . /app

ENV PORT=8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]