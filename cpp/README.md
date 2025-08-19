# C++ Docker開発テンプレート

このテンプレートは、標準的なC++開発環境をDocker Composeで簡単に構築できるプロジェクト雛形です。

## 特長
- UbuntuベースのシンプルなC++開発環境
- ホストユーザーのUID/GIDに合わせたパーミッション設定
- パスワードなしsudo
- .env自動生成スクリプト付き
- サンプルC++プロジェクト(example)付き

## セットアップ手順
1. 必要ツールのインストール
   - Docker
   - Docker Compose

2. このリポジトリをクローン

3. `.env`ファイルの生成(自動)
   ```bash
   ./launch.sh
   ```
   ※初回はイメージビルドとコンテナ起動まで自動実行されます

4. コンテナに入る
   ```bash
   docker compose exec cpp bash
   ```

5. C++プロジェクトのビルド例
   ```bash
   cd src/example
   mkdir -p build && cd build
   cmake .. && make
   ./example
   ```

## srcディレクトリ
- `src/`配下にC++プロジェクトを追加できます
- 例: `src/my_project` など

## 注意事項
- 開発目的のため、コンテナ内での `sudo` はパスワードなしで実行できます(セキュリティに注意)
- Dockerfileやcomposeファイルのバージョンは適宜調整してください

## ライセンス
このプロジェクトはMIT Licenseで公開されています。
詳細はLICENSEファイルを参照してください。
