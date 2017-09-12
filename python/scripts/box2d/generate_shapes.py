#! /usr/bin/python

class Octaeder:
    def __init__(self):
        self._points = [[1.0, 0.0, 0.7071, 0.7071, 0, 1, -0.7071, 0.7071, -1, 0, -0.7071, -0.7071, 0, -1, 0.7071, -0.7071]]

    def get_geometry(self, scale):
        return map(lambda x: map(lambda y: scale * y, x), self._points)

    @staticmethod
    def get_width_height(scale):
        return scale, scale

    @staticmethod
    def get_name():
        return 'Octaeder'


class Box:
    def __init__(self):
        self._points = [[0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0]]

    def get_geometry(self, scale):
        return map(lambda x: map(lambda y: scale * y, x), self._points)

    @staticmethod
    def get_width_height(scale):
        return scale, scale

    @staticmethod
    def get_name():
        return 'Box'


class Rectangle:
    def __init__(self):
        self._points = [[0.0, 0.0, 1.0, 0.0, 1.0, 0.5, 0.0, 0.5]]

    def get_geometry(self, scale):
        return map(lambda x: map(lambda y: scale * y, x), self._points)

    @staticmethod
    def get_width_height(scale):
        return scale, 0.5 * scale

    @staticmethod
    def get_name():
        return 'Rectangle'


class HLetter:
    def __init__(self):
        self._points = [[0.0, 0.0, 0.3, 0.0, 0.3, 0.9, 0.0, 0.9],
                        [0.3, 0.3, 0.6, 0.3, 0.6, 0.6, 0.3, 0.6],
                        [0.6, 0.0, 0.9, 0.0, 0.9, 0.9, 0.6, 0.9]]

    def get_geometry(self, scale):
        return map(lambda x: map(lambda y: scale * y, x), self._points)

    @staticmethod
    def get_width_height(scale):
        return scale * 0.9, scale * 0.9

    @staticmethod
    def get_name():
        return 'HLetter'


class TLetter:
    def __init__(self):
        self._points =[[0.0, 0.6, 0.9, 0.6, 0.9, 0.9, 0.0, 0.9], [0.3, 0.0, 0.6, 0.0, 0.6, 0.6, 0.3, 0.6]]

    def get_geometry(self, scale):
        return map(lambda x: map(lambda y: scale * y, x), self._points)

    @staticmethod
    def get_width_height(scale):
        return scale * 0.9, scale * 0.9

    @staticmethod
    def get_name():
        return 'TLetter'


class KLetter:
    def __init__(self):
        self._points = [[0.0, 0.0, 0.3, 0.0, 0.3, 0.9, 0.0, 0.9],
                        [0.3, 0.3, 0.6, 0.0, 0.9, 0.0, 0.9, 0.3, 0.45, 0.45, 0.3, 0.45],
                        [0.3, 0.6, 0.45, 0.45, 0.9, 0.6, 0.9, 0.9, 0.6, 0.9, 0.3, 0.6]]

    def get_geometry(self, scale):
        return map(lambda x: map(lambda y: scale * y, x), self._points)

    @staticmethod
    def get_width_height(scale):
        return 0.9 * scale, 0.9 * scale

    @staticmethod
    def get_name():
        return 'KLetter'


def get_all_shapes():
    return [Octaeder(), Box(), Rectangle(), HLetter(), TLetter(), KLetter()]
