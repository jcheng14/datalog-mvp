def get_color(index):
    """ Return a color from a set of predefined colors. Contains 80 colors in total.
    code originally from https://github.com/fizyr/keras-retinanet/
    Args
        label: The label to get the color for.
    Returns
        A list of three values representing a RGB color.
    """
    if index < len(colors):
        return colors[index]
    
    
colors = [    
    [1.0, 0.0, 0.0], #Red
    [0.0, 1.0, 0.0], #Green 
    [0.0, 0.0, 1.0], #Blue
    [1.0, 1.0, 0.0], #Yellow 
    [1.0, 0.0, 1.0], #Magenta 
    [0.0, 1.0, 1.0], #Cyan 
    [1.0, 0.5, 0.0], #Orange
    [0.5, 0.0, 0.0], #Dark Red
    [0.0, 0.5, 0.0], #Dark Green
    [0.0, 0.0, 0.5], #Dark Blue
    [0.5, 0.5, 0.0], #Dark Yellow
    [0.5, 0.0, 0.5], #Dark Magenta
    [0.0, 0.5, 0.5], #Dark Cyan
    [0.5, 0.25, 0.0], #Dark Orange
    [1.0, 0.5, 0.5], #Light Red 
    [0.5, 1.0, 0.5], #Light Green
    [0.5, 0.5, 1.0], #Light Blue
    [1.0, 1.0, 0.5], #Light Yellow
    [1.0, 0.5, 1.0], #Light Magenta
    [0.5, 1.0, 1.0], #Light Cyan
    [1.0, 0.75, 0.5], #Light Orange
    [0.25, 0.25, 0.25], #Dark Gray
    [0.75, 0.75, 0.75] #Light Gray
]







