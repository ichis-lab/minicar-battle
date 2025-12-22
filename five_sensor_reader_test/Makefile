# Makefile for five_sensor_reader tests
#
# 使用方法:
#   make              - Dockerコンテナで全テスト実行（デフォルト）
#   make test         - Dockerコンテナで全テスト実行
#   make test-pid     - PIDControllerテストのみ
#   make test-wall    - WallDetectorテストのみ
#   make test-steering - SteeringControllerテストのみ
#   make test-integration - 結合テストのみ
#   make shell        - Dockerシェルを起動
#   make clean        - ビルド成果物を削除

# Docker設定
DOCKER_IMAGE = five_sensor_test
DOCKER_TAG = latest

# コンパイラ設定（コンテナ内用）
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g -O0 -I./src -I./test
GTEST_FLAGS = -lgtest -lgtest_main -pthread

# ディレクトリ
SRC_DIR = src
TEST_DIR = test
BUILD_DIR = build

# ソースファイル
SRCS = $(SRC_DIR)/ArduinoMock.cpp \
       $(SRC_DIR)/PIDController.cpp \
       $(SRC_DIR)/WallDetector.cpp \
       $(SRC_DIR)/SteeringController.cpp

# テストファイル
TEST_SRCS = $(TEST_DIR)/test_pid_controller.cpp \
            $(TEST_DIR)/test_wall_detector.cpp \
            $(TEST_DIR)/test_steering_controller.cpp \
            $(TEST_DIR)/test_integration.cpp

# オブジェクトファイル
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)
TEST_OBJS = $(TEST_SRCS:$(TEST_DIR)/%.cpp=$(BUILD_DIR)/%.o)

# 実行ファイル
TEST_BIN = $(BUILD_DIR)/run_tests

# ============================================================================
# デフォルトターゲット（Docker経由で全テスト実行）
# ============================================================================
.PHONY: all
all: test

.PHONY: test
test: docker-build
	@echo "=== Running all tests in Docker ==="
	@docker run --rm $(DOCKER_IMAGE):$(DOCKER_TAG)

# ============================================================================
# 特定のテストをDocker経由で実行
# ============================================================================
.PHONY: test-pid
test-pid: docker-build
	@echo "=== Running PID Controller tests ==="
	@docker run --rm $(DOCKER_IMAGE):$(DOCKER_TAG) \
		./build/run_tests --gtest_filter="PIDControllerTest.*" --gtest_color=yes

.PHONY: test-wall
test-wall: docker-build
	@echo "=== Running Wall Detector tests ==="
	@docker run --rm $(DOCKER_IMAGE):$(DOCKER_TAG) \
		./build/run_tests --gtest_filter="WallDetectorTest.*" --gtest_color=yes

.PHONY: test-steering
test-steering: docker-build
	@echo "=== Running Steering Controller tests ==="
	@docker run --rm $(DOCKER_IMAGE):$(DOCKER_TAG) \
		./build/run_tests --gtest_filter="SteeringControllerTest.*" --gtest_color=yes

.PHONY: test-integration
test-integration: docker-build
	@echo "=== Running Integration tests ==="
	@docker run --rm $(DOCKER_IMAGE):$(DOCKER_TAG) \
		./build/run_tests --gtest_filter="IntegrationTest.*" --gtest_color=yes

.PHONY: test-verbose
test-verbose: docker-build
	@echo "=== Running all tests (verbose) ==="
	@docker run --rm $(DOCKER_IMAGE):$(DOCKER_TAG) \
		./build/run_tests --gtest_color=yes --gtest_print_time=1

# ============================================================================
# Docker操作
# ============================================================================
.PHONY: docker-build
docker-build:
	@echo "Building Docker image..."
	@docker build -t $(DOCKER_IMAGE):$(DOCKER_TAG) .

.PHONY: shell
shell: docker-build
	@echo "Starting Docker shell..."
	@docker run --rm -it $(DOCKER_IMAGE):$(DOCKER_TAG) /bin/bash

.PHONY: docker-rebuild
docker-rebuild:
	@echo "Rebuilding Docker image (no cache)..."
	@docker build --no-cache -t $(DOCKER_IMAGE):$(DOCKER_TAG) .

# ============================================================================
# ローカルビルド（コンテナ内で使用）
# ============================================================================
.PHONY: build-local
build-local: $(BUILD_DIR) $(TEST_BIN)

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(TEST_BIN): $(OBJS) $(TEST_OBJS)
	@echo "Linking $@"
	@$(CXX) $(CXXFLAGS) -o $@ $^ $(GTEST_FLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	@echo "Compiling $<"
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(TEST_DIR)/%.cpp | $(BUILD_DIR)
	@echo "Compiling $<"
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

.PHONY: run-local
run-local: build-local
	@echo "=== Running tests ==="
	@$(TEST_BIN) --gtest_color=yes

# ============================================================================
# クリーン
# ============================================================================
.PHONY: clean
clean:
	@echo "Cleaning build artifacts"
	@rm -rf $(BUILD_DIR)

.PHONY: docker-clean
docker-clean:
	@echo "Removing Docker image..."
	@docker rmi $(DOCKER_IMAGE):$(DOCKER_TAG) 2>/dev/null || true

# ============================================================================
# カバレッジ（Docker内で実行）
# ============================================================================
.PHONY: coverage
coverage: docker-build
	@echo "=== Running tests with coverage ==="
	@docker run --rm -v $(PWD)/coverage:/app/coverage $(DOCKER_IMAGE):$(DOCKER_TAG) \
		sh -c "make CXXFLAGS='$(CXXFLAGS) --coverage' build-local && \
		       ./build/run_tests && \
		       gcovr --html --html-details -o coverage/index.html"
	@echo "Coverage report: coverage/index.html"

# ============================================================================
# ヘルプ
# ============================================================================
.PHONY: help
help:
	@echo "five_sensor_reader テスト用Makefile"
	@echo ""
	@echo "使用方法（全てDockerコンテナ内で実行）:"
	@echo "  make               - 全テストを実行（デフォルト）"
	@echo "  make test          - 全テストを実行"
	@echo "  make test-verbose  - 詳細出力でテスト実行"
	@echo "  make test-pid      - PIDControllerテストのみ"
	@echo "  make test-wall     - WallDetectorテストのみ"
	@echo "  make test-steering - SteeringControllerテストのみ"
	@echo "  make test-integration - 結合テストのみ"
	@echo "  make shell         - Dockerシェルを起動（デバッグ用）"
	@echo "  make clean         - ローカルビルド成果物を削除"
	@echo "  make docker-clean  - Dockerイメージを削除"
	@echo "  make docker-rebuild - キャッシュなしでDocker再ビルド"
	@echo "  make coverage      - カバレッジレポート生成"
	@echo "  make help          - このヘルプを表示"
