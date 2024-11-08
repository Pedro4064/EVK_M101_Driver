import re

with open('2024-10-21_111422_NMEA_ONLY.ubx', 'r') as log_file:
    file_content = log_file.read()

MESSAGE_ID_REGEX = r'\$(?P<MESSAGE_ID>\w*),'
matches_found = re.findall(MESSAGE_ID_REGEX, file_content)
print(set(matches_found))