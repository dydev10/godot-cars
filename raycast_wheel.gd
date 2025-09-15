extends RayCast3D
class_name RaycastWheel

@export var springStrengh := 100
@export var springDamping := 2.0  # damp coefficient is decided by: d=z(2Sqrt[k*m]) eg: for z=0.1,m=50,k=5000, d-> 31.6228
@export var restDist := 0.5
@export var wheelRadius := 0.4
@export var overExtend := 0.0
@export var isMotor := false
@export var gripCurve: Curve

@onready var wheel: Node3D = get_child(0)  # assumes mesh is the first child node
