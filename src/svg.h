#pragma once

#include <cstdio> // for sprintf
#include <fstream>
#include <iostream>
#include <sstream>

struct RGB {
    double r = 0;
    double g = 0;
    double b = 0;

    RGB()
    {
    }

    RGB(double r, double g, double b) : r(r), g(g), b(b)
    {
    }

    RGB(const RGB& rgb) : r(rgb.r), g(rgb.g), b(rgb.b)
    {
    }

    const RGB& operator=(const RGB& rgb)
    {
        r = rgb.r;
        g = rgb.g;
        b = rgb.b;
        return *this;
    }

    friend RGB operator+(const RGB& a, const RGB& b)
    {
        return RGB(a.r + b.r, a.g + b.g, a.b + b.b);
    }

    friend RGB operator-(const RGB& a, const RGB& b)
    {
        return RGB(a.r - b.r, a.g - b.g, a.b - b.b);
    }

    double sqrt() const
    {
        return r * r + g * g + b * b;
    }

    std::string text() const
    {
        std::stringstream ss;
        ss << "rgb(" << (int)r << "," << (int)g << "," << (int)b << ")";
        return ss.str();
    }
};

RGB getElementColor(int index);
RGB getMixedElementColor(int a, int b);

/// SvgWriter simplifies a process of writing SVG file.
class SvgWriter {
public:
    /// Helper structure to write XML tag to a file
    struct Tag {
        /// Constructor
        /// @param out - output stream
        /// @param name - name of a tag
        /// @param body - does this tag has a specific body.
        /// If tag has no body, then only short form of <name ... /> will be added
        Tag(std::ostream& out, const char* name, bool body = false)
            : out(out), name(name), body(body)
        {
            out << "<" << name;
        }

        Tag(Tag && other) : out(other.out)
        {
            name = std::move(other.name);
            body = other.body;
        }

        ~Tag()
        {
            if (!name.empty()) {
                if (body)
                    out << "</" << name.c_str() << ">" << std::endl;
            }
        }

        /// AttrFinisher writes a proper tag ender after all attributes are written.
        struct AttrFinisher {
            AttrFinisher(std::ostream& out, bool simple) : out(out), simple(simple)
            {
            }

            ~AttrFinisher()
            {
                out << (simple ? "/>" : ">") << std::endl;
            }

            operator std::ostream&()
            {
                return out;
            }

            /// Is it a simple tag ender with <name ... /> or more complex <tag ...> ... </tag>
            bool simple = false;
            std::ostream& out;
        };

        /// Returns ostream-compatible wrapper for writing tag attributes.
        AttrFinisher attrs()
        {
            return AttrFinisher(out, !body);
        }

    protected:
        /// Tag name.
        std::string name;
        /// True if tag has a body.
        bool body = false;
        /// Output stream.
        std::ostream& out;
    };

    /// Helper struct to write attribute values
    struct Attribute {
        std::string name;
        std::string value;

        template <class T>
        Attribute(const char* name, T value) : name(name), value(std::to_string(value))
        {
        }

        Attribute(const char* name, const char* value) : name(name), value(value)
        {
        }

        Attribute(const char* name, const RGB& rgb) : name(name)
        {
            value = rgb.text();
        }

        friend std::ostream& operator<<(std::ostream& out, const Attribute& attr)
        {
            out << " " << attr.name.c_str() << "=\"" << attr.value.c_str() << "\"";
            return out;
        }
    };

    using Attr = Attribute;

    /// Constructor
    /// @param file - path to a svg file
    /// @param width - canvas width
    /// @param height - canvas height
    /// Output file is opened automatically. Final tag will be written in a destructor.
    SvgWriter(const char* file, int width, int height) : fout(file)
    {
        canvasWidth = width;
        canvasHeight = height;
        out() << "<svg " << Attr("width", canvasWidth) << Attr("height", canvasHeight)
              << Attr("xmlns", "http://www.w3.org/2000/svg") << ">" << std::endl;

        out() << "<style>" << std::endl
              << "    text{ font: italic 10px monospace; }" << std::endl
              << "</style>" << std::endl;
    }

    ~SvgWriter()
    {
        out() << "</svg>" << std::endl;
    }

    /// Add rectangle to SVG.
    void rect(int x, int y, int width, int height, const char* fill, const char* style)
    {
        int lineWidth = 1;
        if (x + width > canvasWidth) {
            canvasWidth = x + width + lineWidth;
        }
        if (y + height > canvasHeight) {
            canvasHeight = x + width + lineWidth;
        }

        Tag tag(out(), "rect");
        auto && attrs = tag.attrs();
        attrs << Attr("x", x) << Attr("y", y) << Attr("width", width) << Attr("height", height);
        if (fill)
            attrs << Attr("fill", fill);
        if (style)
            attrs << Attr("style", style);
    }

    /// Draw vertical text.
    void vtext(int x, int y, const char* text, const char* style, const char* anchor = nullptr)
    {
        auto vertical_style = std::string("writing-mode: tb;") + style;
        Tag tag(out(), "text", true);
        {
            auto && attrs = tag.attrs();
            attrs << Attr("x", x) << Attr("y", y) << Attr("style", vertical_style.c_str());
            if (anchor)
                attrs << Attr("text-anchor", anchor);
        }
        out() << text;
    }

    /// Draw horizontal text.
    void htext(int x, int y, const char* text, const char* style, const char* anchor = nullptr)
    {
        Tag tag(out(), "text", true);
        {
            auto && attrs = tag.attrs();
            attrs << Attr("x", x) << Attr("y", y) << Attr("style", style);
            if (anchor)
                attrs << Attr("text-anchor", anchor);
        }
        out() << text;
    }

    /// Draw horizontal line.
    void hline(int x, int y, int w, const char* style)
    {
        Tag tag(out(), "line");
        auto && attrs = tag.attrs();
        attrs << Attr("x1", x) << Attr("y1", y) << Attr("x2", x + w) << Attr("y2", y);
        if (style)
            attrs << Attr("style", style);
    }

    /// Draw a regular line.
    void line(int x0, int y0, int x1, int y1, const char* style)
    {
        Tag tag(out(), "line");
        auto&& attrs = tag.attrs();
        attrs << Attr("x1", x0) << Attr("y1", y0) << Attr("x2", x1) << Attr("y2", y1);
        if (style)
            attrs << Attr("style", style);
    }

    /// Define a reusable symbol
    /// @returns created tag. It should be captured using && object.
    Tag symbol(const char* name, int width, int height)
    {
        Tag tag(out(), "symbol", true);
        tag.attrs() << Attr("id", name) << Attr("width", width) << Attr("height", height);
        return std::move(tag);
    }

    /// Wraps <use> element
    void use(const std::string& name, int x, int y, const char* style = nullptr)
    {
        Tag tag(out(), "use");
        auto attrs = tag.attrs();
        attrs << Attr("href", name.c_str()) << Attr("x", x) << Attr("y", y);
        if (style) {
            attrs << Attr("style", style);
        }
    }

    std::ostream& out()
    {
        return fout;
    }

protected:
    std::ofstream fout;
    int canvasWidth = 0;
    int canvasHeight = 0;
};
