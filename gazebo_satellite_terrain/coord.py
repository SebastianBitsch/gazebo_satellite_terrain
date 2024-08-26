from dataclasses import dataclass

@dataclass(frozen=True, slots=True)
class Coord:
    """
    Kinda unnecessary, but can be used in any python project tbf
    """
    x: float = 0
    y: float = 0

    def __repr__(self) -> str:
        return f"({self.x}, {self.y})"

    def __add__(self, other):
        if isinstance(other, self.__class__):
            return Coord(self.x + other.x, self.y + other.y)
        elif isinstance(other, int | float):
            return Coord(self.x + other, self.y + other)
        else:
            raise TypeError(f"Operand not supported between instances of '{self.__class__}' and '{type(other)}'")

    def __neg__(self):
        return Coord(-self.x, -self.y)

    def __sub__(self, other):
        return self + -other

    def __mul__(self, other):
        if isinstance(other, self.__class__):
            return Coord(self.x * other.x, self.y * other.y)
        elif isinstance(other, int | float):
            return Coord(self.x * other, self.y * other)
        else:
            raise TypeError(f"Operand not supported between instances of '{self.__class__}' and '{type(other)}'")

    def __floordiv__ (self, other):
        if isinstance(other, self.__class__):
            return Coord(self.x // other.x, self.y // other.y)
        elif isinstance(other, int | float):
            return Coord(self.x // other, self.y // other)
        else:
            raise TypeError(f"Operand not supported between instances of '{self.__class__}' and '{type(other)}'")

    def __truediv__ (self, other):
        if isinstance(other, self.__class__):
            return Coord(self.x / other.x, self.y / other.y)
        elif isinstance(other, int | float):
            return Coord(self.x / other, self.y / other)
        else:
            raise TypeError(f"Operand not supported between instances of '{self.__class__}' and '{type(other)}'")

    def __radd__(self, other):
        return self + other

    def __rsub__(self, other):
        return self - other

    def __rmul__(self, other):
        return self * other
    
    def __lt__(self, other):
        if isinstance(other, self.__class__):
            return self.x < other.x and self.y < other.y
        elif isinstance(other, int | float):
            return self.x < other and self.y < other
        else:
            raise TypeError(f"'<' not supported between instances of '{self.__class__}' and '{type(other)}'")

    def __le__(self, other):
        if isinstance(other, self.__class__):
            return self.x <= other.x and self.y <= other.y
        elif isinstance(other, int | float):
            return self.x <= other and self.y <= other
        else:
            raise TypeError(f"'<=' not supported between instances of '{self.__class__}' and '{type(other)}'")

    def __gt__(self, other) -> bool:
        return not self.__lt__(other)
    
    def __ge__(self, other) -> bool:
        return not self.__le__(other)

