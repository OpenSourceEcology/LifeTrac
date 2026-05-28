import sys

p = '/opt/lifetrac/video-test/docker-compose.yml'
try:
    with open(p, 'r') as fh:
        s = fh.read()
    s = s.replace('- LIFETRAC_REG_PROFILE=1', '- LIFETRAC_REG_PROFILE=0')
    s = s.replace('- LIFETRAC_FHSS_WIDE_MASK=1', '# - LIFETRAC_FHSS_WIDE_MASK=1')
    with open(p, 'w') as fh:
        fh.write(s)
    print("SUCCESS: TR_COMP_MODIFIED")
except Exception as e:
    print("ERROR:", e)
