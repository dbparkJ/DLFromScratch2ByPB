from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route('/col/v1/image/saveImageFiles.do', methods=['POST'])
def upload_file():
    if 'images' not in request.files:
        return jsonify({"status": "error", "message": "No file uploaded"}), 400
    
    file = request.files['images']
    box_result = request.args.get('box_result', '')
    timestamp = request.args.get('timestamp', '')

    # 파일 저장 (테스트용)
    file.save(f"./uploaded_files/{file.filename}")
    
    return jsonify({
        "status": "success",
        "message": "File uploaded successfully",
        "box_result": box_result,
        "timestamp": timestamp
    }), 200

if __name__ == '__main__':
    app.run(debug=True, port=5000)
