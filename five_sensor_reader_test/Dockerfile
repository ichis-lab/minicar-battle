# Dockerfile for five_sensor_reader tests
#
# Google Testをインストールした軽量C++テスト環境

FROM ubuntu:22.04

# 環境変数
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

# 必要なパッケージをインストール
RUN apt-get update && apt-get install -y \
    build-essential \
    g++ \
    make \
    cmake \
    gcovr \
    && rm -rf /var/lib/apt/lists/*

# Google Test をソースからビルド
RUN apt-get update && apt-get install -y libgtest-dev && \
    cd /usr/src/gtest && \
    cmake CMakeLists.txt && \
    make && \
    cp lib/*.a /usr/lib/ 2>/dev/null || cp *.a /usr/lib/ 2>/dev/null || true && \
    rm -rf /var/lib/apt/lists/*

# 作業ディレクトリ
WORKDIR /app

# ソースコードをコピー
COPY src/ ./src/
COPY test/ ./test/
COPY Makefile ./

# テストをビルド
RUN make build-local

# デフォルトコマンド: テスト実行
CMD ["./build/run_tests", "--gtest_color=yes"]
