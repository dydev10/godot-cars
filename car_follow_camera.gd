extends Camera3D

@export var minDistance := 4.0
@export var maxDistance := 8.0
@export var height := 3.0
@export var cameraSensitivity := 0.001

@onready var target: Node3D = get_parent().get_parent()


func _input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		top_level = false
		get_parent().rotate_y(-event.relative.x * cameraSensitivity)
		top_level = true


func  _physics_process(delta: float) -> void:
	var fromTarget := global_position - target.global_position
	
	if fromTarget.length() < minDistance:
		fromTarget = fromTarget.normalized() * minDistance
	elif  fromTarget.length() > maxDistance:
		fromTarget = fromTarget.normalized() * maxDistance
		
	fromTarget.y = height
	global_position = target.global_position + fromTarget

	var lookDir := global_position.direction_to(target.global_position).abs() - Vector3.UP
	if not lookDir.is_zero_approx():
		look_at_from_position(global_position, target.global_position, Vector3.UP)
