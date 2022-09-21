#include "documents.h"


const char *Document::toNameFormat(Document::Format format)
{
    switch (format) {
    case Format::Binary: return "BinDocKaanh";
    case Format::Xml: return "XmlDocKaanh";;
    }
    return nullptr;
}

Document::Document(): QObject(nullptr),
      TDocStd_Document("BinDocKaanh")
{
    TDF_TagSource::Set(this->GetData()->Root());
}
