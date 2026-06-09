import onnx
model = onnx.load("best.onnx")
print(model.graph.input[0].type.tensor_type.shape)
