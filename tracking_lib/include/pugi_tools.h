#pragma once

#include <pugixml.hpp>
#include <string>

namespace pugi {
    // Functor for xml child
    class xml_child {
    private:
        pugi::xml_node m_node;
    public:
        xml_child(const pugi::xml_node &a_node) : m_node(a_node) {}

        ~xml_child() {}

        pugi::xml_node operator()(const std::string &name);
    };

    bool load_preprocess(pugi::xml_document &doc, const char *path);

    bool preprocess(pugi::xml_node node);
};
