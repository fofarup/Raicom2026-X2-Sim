"""Non-secret DeepSeek defaults for the real-robot runtime.

Supply the API key through ``DEEPSEEK_API_KEY``. Never commit a real key to
this file; the deployed private backup may keep its own protected copy.
"""

DEEPSEEK_API_KEY = ""
DEEPSEEK_BASE_URL = "https://api.deepseek.com"
DEEPSEEK_MODEL = "deepseek-v4-flash"
DEEPSEEK_TIMEOUT_SECONDS = 2.0
