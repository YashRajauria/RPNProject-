def generate_corridor(grid_map, path, inflation=2, safety_margin=1):

    corridors = []

    width = grid_map.width
    height = grid_map.height


    # SAFETY CHECK
    def is_safe(x, y):
        for dx in range(-safety_margin, safety_margin + 1):
            for dy in range(-safety_margin, safety_margin + 1):
                if not grid_map.is_free(x + dx, y + dy):
                    return False
        return True


    # SEGMENT-BASED CORRIDORS
    for i in range(len(path) - 1):

        x1, y1 = path[i]
        x2, y2 = path[i + 1]


        x1_i, y1_i = int(round(x1)), int(round(y1))
        x2_i, y2_i = int(round(x2)), int(round(y2))

        xmin = min(x1_i, x2_i)
        xmax = max(x1_i, x2_i)
        ymin = min(y1_i, y2_i)
        ymax = max(y1_i, y2_i)



        # EXPANSION

        # LEFT
        while xmin - 1 >= 0 and all(
            is_safe(xmin - 1, yy) for yy in range(ymin, ymax + 1)
        ):
            xmin -= 1

        # RIGHT
        while xmax + 1 < width and all(
            is_safe(xmax + 1, yy) for yy in range(ymin, ymax + 1)
        ):
            xmax += 1

        # DOWN
        while ymin - 1 >= 0 and all(
            is_safe(xx, ymin - 1) for xx in range(xmin, xmax + 1)
        ):
            ymin -= 1

        # UP
        while ymax + 1 < height and all(
            is_safe(xx, ymax + 1) for xx in range(xmin, xmax + 1)
        ):
            ymax += 1

        # add small soft margin
        margin = 0.2
        xmin -= margin
        xmax += margin
        ymin -= margin
        ymax += margin


        # ENSURE MIN SIZE
        min_size = 3.0

        if xmax - xmin < min_size:
            center = (xmin + xmax) / 2
            xmin = center - min_size / 2
            xmax = center + min_size / 2

        if ymax - ymin < min_size:
            center = (ymin + ymax) / 2
            ymin = center - min_size / 2
            ymax = center + min_size / 2

        if len(corridors) > 0:
            prev = corridors[-1]

            xmin = min(xmin, prev[0])
            xmax = max(xmax, prev[1])
            ymin = min(ymin, prev[2])
            ymax = max(ymax, prev[3])

        corridors.append((xmin, xmax, ymin, ymax))

    print(f"[DEBUG] Generated corridors: {len(corridors)}")

    return corridors