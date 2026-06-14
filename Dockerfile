# Auto-Route-Planning + HELIOS++ — fully self-contained image.
# Build once, copy/run on any machine with Docker (no manual HELIOS++ setup).
FROM python:3.11-slim

# - libgomp1: OpenMP runtime required by the HELIOS++ binary
# - libexpat1: XML parser needed by rasterio/GDAL (not in python:slim)
# - bash, ca-certificates: required by the HELIOS++ self-extracting installer
# - build-essential: fallback for any Python deps without prebuilt wheels
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgomp1 \
    libexpat1 \
    bash \
    ca-certificates \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Bake a Linux HELIOS++ install into the image (downloads the latest release
# from GitHub at build time, so this step requires network access). Done
# before COPY . . so app-code edits don't invalidate this ~290MB layer.
COPY src/helios_setup.py src/helios_setup.py
RUN python -c "import sys; sys.path.insert(0, 'src'); import helios_setup; helios_setup.download_and_install(print)"

COPY . .

EXPOSE 8501

CMD ["streamlit", "run", "app.py", "--server.address=0.0.0.0", "--server.port=8501", "--server.headless=true"]
