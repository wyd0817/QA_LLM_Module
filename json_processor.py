import json
import re
from loguru import logger

class JsonProcessor:
    def process_response(self, response):
        try:
            # Search for JSON string in the response
            json_str_match = re.search(r'\{.*\}', response, re.DOTALL)
            if json_str_match:
                # Get the matched JSON string
                json_str = json_str_match.group()
                logger.debug(f"Full JSON string: {json_str}")  

                # Replace escape characters and remove trailing commas
                json_str = json_str.replace("\\", "")
                json_str = json_str.replace(r'\\_', '_')
                json_str = re.sub(r',\s*}', '}', json_str)
                json_str = re.sub(r',\s*\]', ']', json_str)

                # Parse the JSON string
                response_json = json.loads(json_str)
                return response_json
            else:
                logger.error("No JSON string match found in response.")
                return None

        except json.JSONDecodeError as e:
            logger.error(f"JSONDecodeError: {e}")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")

        return None
