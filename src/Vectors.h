#pragma once
#include <cmath>
#include <iostream>

class Vector3f {
public:
    double x, y, z;
    Vector3f(double _x = 0, double _y = 0, double _z = 0) : x(_x), y(_y), z(_z) {}

    Vector3f operator+(const Vector3f &b) const {
        return Vector3f(x + b.x, y + b.y, z + b.z);
    }
    Vector3f operator-(const Vector3f &b) const {
        return Vector3f(x - b.x, y - b.y, z - b.z);
    }
    Vector3f operator*(const Vector3f &b) const {
        return Vector3f(x * b.x, y * b.y, z * b.z);
    } // pointwise product
    Vector3f operator*(double b) const {
        return Vector3f(x * b, y * b, z * b);
    }
    friend Vector3f operator*(double b, Vector3f v);
    Vector3f normalized() {
        return (*this) * (1 / sqrt(x*x + y*y + z*z));
    }
    double dot(const Vector3f &b) const {
        return x * b.x + y * b.y + z * b.z;
    }
    friend double dot(const Vector3f &a, const Vector3f &b);
    Vector3f cross(const Vector3f &b) const {
        return Vector3f(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
    }
    friend Vector3f cross(const Vector3f &a, const Vector3f &b);
    Vector3f pow(const double b) const {
        return Vector3f(std::pow(x, b), std::pow(y, b), std::pow(z, b));
    }
    friend Vector3f pow(const Vector3f &a, double b);
    Vector3f rotate(Vector3f axis, double theta) const {
        // the following function implements a 3d rotation
        // referenced Raina's implementation
        Vector3f ret;
        double cost = cos(theta);
        double sint = sin(theta);
        ret.x += x * ( axis.x * axis.x + ( 1 - axis.x * axis.x ) * cost );
        ret.x += y * ( axis.x * axis.y * ( 1 - cost ) - axis.z * sint );
        ret.x += z * ( axis.x * axis.z * ( 1 - cost ) + axis.y * sint );
        ret.y += x * ( axis.y * axis.x * ( 1 - cost ) + axis.z * sint );
        ret.y += y * ( axis.y * axis.y + ( 1 - axis.y * axis.y ) * cost );
        ret.y += z * ( axis.y * axis.z * ( 1 - cost ) - axis.x * sint );
        ret.z += x * ( axis.z * axis.x * ( 1 - cost ) - axis.y * sint );
        ret.z += y * ( axis.z * axis.y * ( 1 - cost ) + axis.x * sint );
        ret.z += z * ( axis.z * axis.z + ( 1 - axis.z * axis.z ) * cost );
        return ret;
    }
    double norm() const {
        return sqrt(x*x + y*y + z*z);
    }
    double inf_norm() const {
        return fmax(fabs(x), fmax(fabs(y), fabs(z)));
    }
    
    friend std::istream &operator >>(std::istream &fin, Vector3f &s);
    friend std::ostream &operator <<(std::ostream &fout, const Vector3f &s);
};

Vector3f operator*(double b, Vector3f v) {
    return Vector3f(v.x * b, v.y * b, v.z * b);
}
double dot(const Vector3f &a, const Vector3f &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vector3f cross(const Vector3f &a, const Vector3f &b) {
    return Vector3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
Vector3f pow(const Vector3f &a, double b) {
    return Vector3f(std::pow(a.x, b), std::pow(a.y, b), std::pow(a.z, b));
}
std::istream &operator >>(std::istream &fin, Vector3f &v) {
    fin >> v.x >> v.y >> v.z;
    return fin;
}
std::ostream &operator <<(std::ostream &fout, const Vector3f &v) {
    fout << "Vector3f(" << v.x << ", " << v.y << ", " << v.z << ")";
    return fout;
}

class Vector2f {
public:
    double x, y;
    Vector2f(double _x = 0, double _y = 0) : x(_x), y(_y) {}

    Vector2f operator+(const Vector2f &b) const {
        return Vector2f(x + b.x, y + b.y);
    }
    Vector2f operator-(const Vector2f &b) const {
        return Vector2f(x - b.x, y - b.y);
    }
    Vector2f operator*(const Vector2f &b) const {
        return Vector2f(x * b.x, y * b.y);
    } // pointwise product
    Vector2f operator*(double b) const {
        return Vector2f(x * b, y * b);
    }
    friend Vector2f operator*(double b, Vector2f v);
    Vector2f normalized() {
        return (*this) * (1 / sqrt(x*x + y*y));
    }
    double dot(const Vector2f &b) const {
        return x * b.x + y * b.y;
    }
    friend double dot(const Vector2f &a, const Vector2f &b);
    double cross(const Vector2f &b) const {
        return x * b.y - y * b.x;
    }
    friend double cross(const Vector2f &a, const Vector2f &b);
    Vector2f pow(const double b) const {
        return Vector2f(std::pow(x, b), std::pow(y, b));
    }
    friend Vector2f pow(const Vector2f &a, double b);
    double norm() const {
        return sqrt(x*x + y*y);
    }
    
    friend std::istream &operator >>(std::istream &fin, Vector2f &s);
    friend std::ostream &operator <<(std::ostream &fout, const Vector2f &s);
};

Vector2f operator*(double b, Vector2f v) {
    return Vector2f(v.x * b, v.y * b);
}
double dot(const Vector2f &a, const Vector2f &b) {
    return a.x * b.x + a.y * b.y;
}
double cross(const Vector2f &a, const Vector2f &b) {
    return a.x * b.y - a.y * b.x;
}
Vector2f pow(const Vector2f &a, double b) {
    return Vector2f(std::pow(a.x, b), std::pow(a.y, b));
}
std::istream &operator >>(std::istream &fin, Vector2f &v) {
    fin >> v.x >> v.y;
    return fin;
}
std::ostream &operator <<(std::ostream &fout, const Vector2f &v) {
    fout << "Vector2f(" << v.x << ", " << v.y << ")";
    return fout;
}