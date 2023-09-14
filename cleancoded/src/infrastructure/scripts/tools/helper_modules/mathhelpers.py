relative = lambda landmark, shape: (int(landmark.x * shape[1]), int(landmark.y * shape[0]))
relativeT = lambda landmark, shape: (int(landmark.x * shape[1]), int(landmark.y * shape[0]), 0)
rel = lambda landmarklist: landmarklist+(0,)
normalize_points = lambda points,  image_height, image_width: [(2 * points[0] / image_width) - 1, (2 * points[1] / image_height) - 1]