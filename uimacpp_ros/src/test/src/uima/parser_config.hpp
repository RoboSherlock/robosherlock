/** \file parser_config.hpp .

 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.


   \brief  Configuration class for parsers.

-------------------------------------------------------------------------- */
#ifndef UIMA_PARSER_CONFIG_HPP
#define UIMA_PARSER_CONFIG_HPP


// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include "uima/pragmas.hpp" //must be first to surpress warnings
#include <map>
#include <stack>
#include <utility>
#include "uima/typesystem.hpp"

namespace uima {

  class TCAS;
  class TextAnalysisEngineSpecifier;

  /**
   * The class ParserConfiguration is used to instruct a parser that puts
   * tagged document into a CAS on how to map document information contained
   * in tags to annotation and features of annotation.
   * This configuration is mainly intended to configure HTML and XML parsers
   * but could be used for any format that uses labeled tags and tag attributes
   *
   * For XML input this configuration object also allows to specify if and how
   * multiple documents are delimited within one physical XML file.
   * Also the content of certain tags can be excluded from being processed at
   * all.
   */
  class ParserConfiguration {
  public:
    ParserConfiguration();
    /**
     * Initialize this config object from settings specified in
     * the parser configuration file.
     *
     * @param parserConfigFilename   file name of the configuration file
     * @param cas                    CAS object defining the types relevant
     *                               for the tag2type mappings
     *
     * @return UIMA_ERR_NONE if OK
     */
    TyErrorId init(icu::UnicodeString const & parserConfigFilename, CAS & cas, ErrorInfo & err);

    /**
     * Returns the tag that is used to delimit/separate multiple
     * logical documents within the same physical file.
     * If no such tag is defined - i.e. there is only one document
     * per file - an empty string is returned.
     *
     * Note: This option will always return the empty string for HTML.
     */
    icu::UnicodeString const & getDocumentDelimiterTag() const;

    /**
     * Sets the tag that is used to delimit/separate multiple
     * logical documents within the same physical file.
     *
     * @see getDocumentDelimiterTag
     */
    void setDocumentDelimiterTag(icu::UnicodeString const & tag);


    /**
     * Returns the tags that are to be ignored when parsing an (XML) document.
     *
     * Only the textual content of tags not included in the returned container
     * will be part of the CAS document.
     */
    vector<icu::UnicodeString> const & getExcludedTags() const;

    /**
     * Sets the tags that are to be ignored when parsing an (XML) document.
     *
     * @see getExcludedTags
     */
    void setExcludedTags( vector<icu::UnicodeString> const & tags);

    /**
     * Returns the type of the annotation that the parser is supposed to
     * create for each occurrence of a tag with name <code>tagName</code>.
     * If no annotation is to be created for occurrences of this tag
     * <code>tagName</code> the return value is an invalid type object.
     *
     * This function returns the value of getDefaultTypeForTags if no
     * explicit mapping has been specified.
     *
     * Note: The returned Type object will be subsumed by type Annotation.
     *
     * @param tagName    The name of the tag to look up
     *
     * @return           The mapped type for tagName or an invalid type
     *                   if no mapping specified
     *
     * @see setDefaultTypeForTags
     */
    Type getTypeForTag(icu::UnicodeString const & tagName) const;

    /**
     * Sets the type of the annotation that the parser is supposed to
     * create for each occurrence of a tag with name <code>tagName</code>.
     *
     * @param tagName    the name of a tag to map
     * @param type       a CAS type (must be valid!)
     *
     * @see getTypeForTag
     */
    void setTypeForTag(icu::UnicodeString const & tagName, Type type);


    /**
     * This option can be used to specify a default mapping in case no
     * explicit mapping is available for a tag.
     *
     * If the default type is set (is valid) getTypeForTag will return
     * this default type whenever no explicit mapping is specified.
     * In this case every tag will be mapped to some type without
     * having to specify many mappings.
     *
     * Since this default value is optional the result of
     * getDefaultTypeForTags() may be invalid.
     * In this case some tags are not mapped to types.
     *
     * @param type    The type to use as default mapping type
     *
     * @see getTypeForTag
     * @see getDefaultTypeForTags
     *
     */
    void setDefaultTypeForTags(Type type);

    /**
     * @return  The default type to use for tags not directly mapped
     *          by getTypeForTag() (may be invalid if none specified)
     *
     * @see setDefaultType
     */
    Type getDefaultTypeForTags() const;

    /**
     * This allows to specify a feature where the name of the tag is stored
     * for each annotation created by the parser.
     * If this feature is invalid the parser will take no action.
     *
     * @param f    The feature where the tag name will stored
     *             f must be of type string.
     *             Also f must be appropriate for all types mapped to
     *             tag.
     *
     * @see getFeatureForTagName
     */
    void setFeatureForTagName(Feature f);

    /**
     * @return  The feature where the name of the tag is stored for each
     *          annotation created by the parser (may be invalid if none
     *          specified).
     *
     * @see setFeatureForTagName
     */
    Feature getFeatureForTagName() const;

    /**
     * For a given tag name and attribute name this function returns an
     * annotation type and a feature of that type to which the value
     * of the attribute is to be mapped.
     *
     * The returned type and feature will be invalid if the attribute
     * <code>attrName</code> at tag <code>tagName</code> is not to be
     * mapped to the CAS.
     *
     * If the returned type and feature are valid the parser is supposed to
     * look for the "last" annotation of the returned type and set the
     * value of the returned feature at this annotation to the value of
     * the attribute <code>attrName</code>.
     *
     * Note that the parser does not necessarily have to create an annotation
     * of the returned type. This mapping can be used to set features of
     * existing annotation: E.g. the attribute "name" of the "meta" tag
     * in HTML could be mapped to the feature "DocumentName" of
     * type "Document"
     *
     * The "last" occurrence of an annotation of the returned type is
     * determined by starting with the annotation corresponding the current
     * tag (if there is one) and searching from there towards the beginning
     * of the text. The annotation corresponding the current tag is included
     * in the search.
     *
     * For each occurrence of a tag the parser is supposed to first check
     * the function <code>getTypeForTag()</code> and create a corresponding
     * annotation if <code>getTypeForTag()</code> returns a valid type.
     *
     * Only after annotation are created for mapped tags the attributes are
     * being mapped to features.
     * This execution order guarantees that for tags that are used in
     * <code>getTypeForTag()</code> and in <code>getFeatureForAttribute()</code>
     * the "last" annotation will be the newly created one.
     *
     * The returned Type object must be subsumed by type Annotation.
     *
     * The returnedFeature must be of type string, integer or float.
     * A conforming parser is supposed to convert the attribute value
     * from it's string representation to an appropriate value before
     * setting the feature value.
     *
     * taph 04.12.2002:  There is currently a limitation for returnedFeature
     * to be of type string only. This will be removed in the future.
     *
     * @param tagName         The name of the tag to look up
     * @param attrName        The name of the attribute of tag tagName to look up
     * @param returnedType    Output param: the type corresponding to tag tagName
     * @param returnedFeature Output param: the feature corresponding to attrName
     */
    void getFeatureForAttribute(
      icu::UnicodeString const & tagName,
      icu::UnicodeString const & attrName,
      Type     & returnedType,
      Feature  & returnedFeature
    ) const;


    /**
     * The break properties that can be specified in getBreakPropertyForTag()
     */
    enum EnBreakProperty {
      enNoBreak                  = 0,
      enWordBreak                = 0x200B,
      enSentenceBreak            = 0x2029,
      enLineBreak                = 0x2028,
      enParagraphBreak           = 0x2029,
      enNumberOfBreakProperties  = 6
    };
    /**
     * Returns the break property the parser is supposed to
     * associate for each occurrence of a tag with name <code>tagName</code>.
     *
     * You can think of break properties as instructions on how to replace
     * a tag with white space content during de-tagging an HTML/XML document.
     * - tags with enNoBreak property will be replaced by the empty string
     *   (e.g. bold &lt;b>F&lt;/b>irst Letter becomes
     *   First Letter)
     * - tags with enWordBreak property will be replaced by a Unicode
     *   U+200B ZERO WIDTH SPACE (e.g. &lt;label>)
     * - tags with enSentenceBreak property will be replaced by a Unicode
     *   paragraph separator character U+2029 PARAGRAPH SEPARATOR ???
     * - tags with enLineBreak property will be replaced by a Unicode
     *   line separator character U+2028 LINE SEPARATOR
     *   (e.g. &lt;br> and &lt;li>)
     * - tags with enParagraphBreak property will be replaced by a Unicode
     *   paragraph separator character U+2029 PARAGRAPH SEPARATOR
     *   (e.g. headings like &lt;h1>)
     *
     * Note that all break properties only apply to the end tag.
     * The begin tag is always replaced by the empty string.
     * For HTML tags that don't have a end tag (e.g. &lt;br>, or where the
     * end tag is optional (e.g. &lt;li>) the parser should introduce the
     * replacement character before the next opening tag that can
     * conceptually close the tag (e.g. &lt;li>) or the next end tag that
     * closes the tag (e.g. &lt;/li>)
     *
     * If no explicit break property has been given
     * getDefaultBreakProperty() is returned.
     *
     * @param tagName    The name of the tag to look up
     */
    EnBreakProperty
    getBreakPropertyForTag(icu::UnicodeString const & tagName) const;

    /**
     * Returns the default break property for tags where no explicit
     * break property has been configured.
     */
    EnBreakProperty getDefaultBreakProperty() const;

    /**
     * Sets the default break property for tags where no explicit
     * break property has been configured.
     */
    void setDefaultBreakProperty(EnBreakProperty enBreakProp);

    /**
     * Returns the (annotation) type corresponding to a break property.
     * The return value may be invalid if no specific type is set.
     * If a type is specified a conforming parser is supposed to
     * create an annotation of that type for each annotation with a given
     * break property (in addition to inserting the corresponding break
     * character)
     *
     * In general this is only usefull for paragraphs.
     * For all other break types (especialy tokens and sentences)
     * a parser can not and should not directly create the entity
     * corresponding to the break (token, sentence) as annotation.
     * The reason for this that the parser only knows that
     * such an annotation must begin at the position of the begin tag and
     * end at the begin tag but not how many entities (tokens, sentences)
     * may be spanned by the tag (e.g. an h1 tag).
     * In general the parser should not create a few instances of
     * annotation types (like tokens or sentences) that are mainly created
     * by annotators (like the tokenizer). Instead it should leave traces in
     * the text that guide the downstream annotator.
     *
     * But paragraphs are an exception to this as they should be
     * created by the parser and only by the parser. For HTML a tokenizer
     * applying plain text paragraph finding heuristics (double newline) would
     * produce incorrect results.
     * So for HTML it would make sense to specify that tags like &lt;p> and &lt;h1>
     * etc. are paragraph break and specify the appropriate paragraph
     * type as the value of getTypeForBreakProperty()
     * Note that you should not map &lt;p> to a type if you do that otherwise
     * two annotation would be created for each &lt;p> tag.
     * If this is done then to ensure consistency <em>only</em> the parser
     * should create paragraph and no annotator should try to do this.
     */
    Type getTypeForBreakProperty(EnBreakProperty enBreakProp) const;
  private:
    // -----------------------------------------------------------------------
    // data structures to store mapping information between XML and UIMA

    // map holding information about XML Elements which are mapped to UIMA
    // types (new annotation are created for each occurrence of such an element)
    typedef map< icu::UnicodeString, Type, less<icu::UnicodeString> >
    TyXMLNameToTypeMap;

    // map holding information about XML Attributes which are mapped to UIMA
    // attributes (UIMA Attributes of existing annotation are set to the values
    // of the XML Attributes)
    // Since each attribute (XML or UIMA) occurs at a certain anchor
    // (XML Element or UIMA Type) we need to store 4 pieces of information
    // Since more than one attr can be mapped from the same element we need a vector
    typedef struct StFeatureInfo_ {
      Type    type;    // 3: Type for Feature
      Feature feature; // 4: Feature
    }
    StFeatureInfo;

    typedef map< icu::UnicodeString, StFeatureInfo, less<icu::UnicodeString> >
    TyXMLAttrToFeatureMap;

    // actual map from 1: the name of XML Element and
    // 2: the XML Attribute name to the rest of the mapping info
    typedef map< icu::UnicodeString, TyXMLAttrToFeatureMap, less<icu::UnicodeString> >
    TyXMLNameToAttrMap;

    // map holding information about XML Elements and their break properties
    typedef map< icu::UnicodeString, EnBreakProperty, less<icu::UnicodeString> >
    TyXMLNameToBreakMap;

    // map holding information about XML Elements and their break properties
    typedef map< EnBreakProperty, Type, less<EnBreakProperty> >
    TyBreakToTypeMap;

    TyXMLNameToTypeMap         iv_mapXMLNameToType;
    TyXMLNameToAttrMap         iv_mapXMLNameToAttr;
    TyXMLNameToBreakMap        iv_mapXMLNameToBreak;
    TyBreakToTypeMap           iv_mapBreakToType;

    icu::UnicodeString         iv_ustrDocumentDelimiterTag;
    vector<icu::UnicodeString> iv_vecustrExcludedTags;

    EnBreakProperty            iv_enDefaultBreakProperty;

    Type                       iv_defaultTypeForTags;
    Feature                    iv_featureForTagName;

  }
  ; /* ParserConfiguration */

} // namespace uima

#endif //UIMA_PARSER_CONFIG_HPP
