#pragma once

#include "pugixml.hpp"
#include "pugi_tools.h"

#include <iostream>
#include <string>
#include <memory>

template<class T>
class abstract_factory_creator {
protected:
    std::shared_ptr<abstract_factory_creator> next;

public:
    abstract_factory_creator() : next(nullptr) {}

    virtual std::shared_ptr<T> create(const pugi::xml_node &node) = 0;

    void add(std::shared_ptr<abstract_factory_creator> ptr) {
        if (next) next->add(ptr);
        else next = ptr;
    }
};


template<class T, class P>
class factory_creator : public abstract_factory_creator<T> {
    std::string m_node_name;

public:
    factory_creator(const std::string &node_name) : m_node_name(node_name) {}

    std::shared_ptr<T> create(const pugi::xml_node &node) override {
        std::shared_ptr<abstract_factory_creator<T> > temp = abstract_factory_creator<T>::next;
        pugi::xml_child child(node);

        std::cout << "Checking if it is " << m_node_name << std::endl;

        if (child("type").child_value() == m_node_name) {
            std::cout << "Yes, it is. Creating" << std::endl;
            auto ptr = std::make_shared<P>();
            ptr->parse(node);
            return ptr;
        } else if (temp) return temp->create(node);
        else return nullptr;
    }
};