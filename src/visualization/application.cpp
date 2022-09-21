#include "application.h"


const Handle(Application)& Application::instance()
{
    static Handle(Application) appPtr;
    if (appPtr.IsNull()) {
        appPtr = new Application;
//        const char strFougueCopyright[] = "Copyright (c) 2021, Fougue Ltd. <http://www.fougue.pro>";
//        appPtr->DefineFormat(
//                    Document::NameFormatBinary, qUtf8Printable(tr("Binary Mayo Document Format")), "myb",
//                    new Document::FormatBinaryRetrievalDriver,
//                    new BinXCAFDrivers_DocumentStorageDriver);
//        appPtr->DefineFormat(
//                    Document::NameFormatXml, qUtf8Printable(tr("XML Mayo Document Format")), "myx",
//                    new Document::FormatXmlRetrievalDriver,
//                    new XmlXCAFDrivers_DocumentStorageDriver(strFougueCopyright));

//        qRegisterMetaType<TreeNodeId>("Mayo::TreeNodeId");
//        qRegisterMetaType<TreeNodeId>("TreeNodeId");
//        qRegisterMetaType<DocumentPtr>("Mayo::DocumentPtr");
//        qRegisterMetaType<DocumentPtr>("DocumentPtr");
    }

    return appPtr;
}

Application::~Application()
{

}

int Application::documentCount() const
{
    return this->NbDocuments();
}

void Application::NewDocument(const TCollection_ExtendedString &format, opencascade::handle<TDocStd_Document> &outDoc)
{
    Handle(Document) newDoc = new Document;
    CDF_Application::Open(newDoc); // Add the document in the session
    outDoc = newDoc;
}

void Application::InitDocument(const opencascade::handle<TDocStd_Document> &doc) const
{
    TDocStd_Application::InitDocument(doc);
    XCAFApp_Application::GetApplication()->InitDocument(doc);
}



Handle(Document) Application::newDocument(Document::Format docFormat)
{
    const char* docNameFormat = Document::toNameFormat(docFormat);
    Handle_TDocStd_Document stdDoc;
    this->NewDocument(docNameFormat, stdDoc);
    return Handle(Document)::DownCast(stdDoc);
}
