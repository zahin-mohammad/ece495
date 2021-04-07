FROM debian:stretch-slim

RUN apt-get update && \
    apt-get install -y \
        build-essential cmake git \
        libuv1-dev libssl-dev libz-dev && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir /build && cd /build && \
    git clone https://github.com/uWebSockets/uWebSockets && \
    cd uWebSockets && \
    git checkout e94b6e1 && \
    mkdir build && cd build && \
    cmake .. && make && make install && \
    ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so && \
    cd / && rm -r /build

EXPOSE 4567

CMD ["sh", "-c", "mkdir -p /app/build && cd /app/build/ && cmake .. && make && ./path_planning"]

