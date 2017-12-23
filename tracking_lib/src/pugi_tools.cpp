#include "pugi_tools.h"
#include <string.h>
#include <sstream>

namespace pugi
{
	pugi::xml_node xml_child::operator ()(const std::string &name)
	{
		pugi::xml_node temp = m_node.child(name.c_str());
		if(!temp)
		{
			std::stringstream err;
			err << "Error: xml parsing was failed: " << name << " node was not found";
			throw std::runtime_error(err.str());
		}
		return temp;
	}

  bool load_preprocess(pugi::xml_document& doc, const char* path)
  {
    pugi::xml_parse_result result = doc.load_file(path, pugi::parse_default | pugi::parse_pi); // for <?include?>

    return result ? preprocess(doc) : false;
  }

  bool preprocess(pugi::xml_node node)
  {
    for (pugi::xml_node child = node.first_child(); child; )
    {
      if (child.type() == pugi::node_pi && strcmp(child.name(), "include") == 0)
      {
        pugi::xml_node include = child;

        // load new preprocessed document (note: ideally this should handle relative paths)
        const char* path = include.value();

        pugi::xml_document doc;
        if (!(pugi::load_preprocess(doc, path))) return false;

        // insert the comment marker above include directive
        node.insert_child_before(pugi::node_comment, include).set_value(path);

        // copy the document above the include directive (this retains the original order!)
        for (pugi::xml_node ic = doc.first_child(); ic; ic = ic.next_sibling())
        {
          node.insert_copy_before(ic, include);
        }

        // remove the include node and move to the next child
        child = child.next_sibling();

        node.remove_child(include);
      }
      else
      {
        if (!preprocess(child)) return false;

        child = child.next_sibling();
      }
    }

    return true;
  }
};
