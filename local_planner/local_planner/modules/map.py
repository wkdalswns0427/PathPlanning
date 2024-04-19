from local_planner.utils.imports import *

class MapDimension(type):
    _map_yaml = os.path.join(os.path.join(os.environ["GOLE_CORE_CONFIG"], "map.yaml"))

    def __new__(meta, name, bases, namespace, version):
        with open(meta._map_yaml, 'r') as f:
            map_vars = yaml.load(f, Loader=yaml.FullLoader)

        if version == 'current':
            version = map_vars['Version']

        dimensions = map_vars['Dimensions'][version]
        landmarks = map_vars['Landmarks']

        namespace.update(dimensions)
        namespace['Landmarks'] = landmarks

        return super().__new__(meta, name, bases, namespace)

class SoccerMap(metaclass=MapDimension, version='current'): # TODO: RC-159
    """
    Map frame:
        - SoccerMap's origin is the very left-bottom corner outside the border
        - coordinate is defined as [x, y, theta]
        - X-axis: to the right from the origin
        - Y-axis: to the top from the origin
        - theta: CCW from the X-axis
    """
    binary_map: np.ndarray = None
    goalpost_width = 0.1

    @classmethod
    def make_discrete_map(cls, resolution:float=0.01, regenerate:bool=False):
        if not regenerate and cls.binary_map is not None:
            return cls.binary_map

        outer_length = cls.FieldLength + 2 * cls.BorderStripWidth
        outer_width = cls.FieldLength + 2 * cls.BorderStripWidth

        data_width = int(outer_width / resolution)
        data_length = int(outer_length / resolution)
        cls.binary_map = np.zeros((data_width, data_length), dtype=bool)

        # Mark Goal Posts
        goalpost_positions = np.array([
            [              0, cls.FieldWidth / 2 - cls.GoalWidth / 2], # left goalpost bottom
            [              0, cls.FieldWidth / 2 + cls.GoalWidth / 2], # left goalpost top
            [cls.FieldLength, cls.FieldWidth / 2 - cls.GoalWidth / 2], # right goalpost bottom
            [cls.FieldLength, cls.FieldWidth / 2 + cls.GoalWidth / 2], # right goalpost top
        ]) + cls.BorderStripWidth

        for gdx in range(goalpost_positions.shape[0]):
            cls.add_obstacle(cls.binary_map, resolution, goalpost_positions[gdx, :], cls.goalpost_width)

        return cls.binary_map

    @staticmethod
    def add_obstacle(map:np.ndarray, resolution:float, position:Iterable, diameter:float):
        bound_px = np.array([-1., 1.])
        x_bound_px = np.round((bound_px * diameter + position[0]) / resolution).astype(np.int64)
        y_bound_px = np.round((bound_px * diameter + position[1]) / resolution).astype(np.int64)

        map[y_bound_px[0]:y_bound_px[1], x_bound_px[0]:x_bound_px[1]] = 1