#!/bin/bash
echo "UID=$(id -u)" > .env
echo "GID=$(id -g)" >> .env
echo ".envファイルを生成しました。"
docker compose up -d

# docker compose exec cpp bash
