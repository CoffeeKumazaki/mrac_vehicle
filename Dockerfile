FROM python:3.7
USER root

WORKDIR /usr/src/app

RUN apt-get update
RUN apt-get install -y build-essential gfortran libopenblas-dev libsuitesparse-dev

ENV CPPFLAGS="-I/usr/include/suitesparse"
RUN pip install --upgrade pip
RUN pip install --upgrade setuptools

RUN python -m pip install jupyterlab

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .
RUN pip install -e adaptive_system
RUN pip install -e vehicle_dynamics
RUN pip install -e util